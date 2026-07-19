use nalgebra::{Matrix3, SMatrix, SMatrixView, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
const CALIBRATION_SWEEPS: usize = 20;
const FACTOR_REGULARIZATION: f32 = 1.0e-4;
const MAX_PARAMETER_STEP: f32 = 0.25;
const MIN_MAG_NORM: f32 = 0.4;

/// Alternating block-coordinate descent for estimating a hard-iron offset and
/// a full SPD soft-iron correction from a fixed, diverse sample buffer.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 3>,
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    hard_iron_offset: Vector3<f32>,
    inverse_soft_iron_cholesky: Matrix3<f32>,
    calibration_initialized: bool,
    mean_distance: f32,
    k: usize,
    max_sample_lifespan_us: u64,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::zeros(),
            sample_timestamps_us: [0; N],
            matrix_filled: Default::default(),
            hard_iron_offset: Vector3::zeros(),
            inverse_soft_iron_cholesky: Matrix3::identity(),
            calibration_initialized: false,
            mean_distance: Default::default(),
            k: 2, // Works well in testing
            max_sample_lifespan_us: 60 * 60 * 1_000_000,
        }
    }
}

impl<const N: usize> MagCalibrator<N> {
    /// Create a new calibrator instance.
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the number of `k` neighbors to calculate distance to.
    pub fn num_neighbors(self, k: usize) -> Self {
        Self {
            k: k.clamp(1, N.saturating_sub(1).max(1)),
            ..self
        }
    }

    /// Configure the maximum time a sample remains in the calibration buffer,
    /// in microseconds. The default is one hour.
    pub fn max_sample_lifespan_us(self, max_sample_lifespan_us: u64) -> Self {
        Self {
            max_sample_lifespan_us,
            ..self
        }
    }

    /// Calculates mean distance to the `k` nearest neighbors.
    /// A smaller number means the point is "similar" to its neighbors.
    fn mean_distance_from_single(&self, vec: SMatrix<f32, 1, 3>) -> f32 {
        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);

        // Distance to every other point
        let mut squared_dists: [f32; N] = [0.; N];
        matrix_view.row_iter().enumerate().for_each(|(j, cmp)| {
            let diff = vec - cmp;
            squared_dists[j] = diff.dot(&diff).sqrt(); // ?
        });

        // Sort floats and return mean distance to nearest neighbors
        squared_dists.sort_unstable_by(|a, b| a.total_cmp(b));
        let k = self.k.min(N.saturating_sub(1));
        if k == 0 {
            return f32::INFINITY;
        }
        squared_dists
            .iter()
            .skip(1)
            .take(k)
            .rfold(0., |a, &b| a + b)
            / k as f32
    }

    /// Calculates mean squared distance to the `k` nearest neighbors
    /// between all `N` row vectors in the internal buffer.
    /// A smaller number means a point is "similar" to its neighbors.
    fn mean_distance_from_all(&self) -> [f32; N] {
        let mut mean_dist: [f32; N] = [0.; N];

        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);
        matrix_view.row_iter().enumerate().for_each(|(i, row)| {
            mean_dist[i] = self.mean_distance_from_single(row.into());
        });
        mean_dist
    }

    /// Returns index of vector with the lowest squared distance
    /// Is used when replacing the least useful value in the array.
    fn lowest_mean_distance_by_index(&mut self) -> (usize, f32) {
        let mean_dist = self.mean_distance_from_all();

        // Set mean distance now that we are at it
        self.mean_distance = mean_dist.iter().rfold(0., |a, &b| a + b) / N as f32;

        // Obtain index for lowest mean distance
        mean_dist
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(index, value)| (index, *value))
            .unwrap()
    }

    /// Evaluates whether the new sample should replace one already in the buffer.
    pub fn evaluate_sample(&mut self, x: [f32; 3], timestamp_us: u64) {
        self.evaluate_sample_vec(Vector3::from(x), timestamp_us)
    }

    /// Add a sample if it is deemed more useful than the least useful sample.
    pub fn evaluate_sample_vec(&mut self, x: Vector3<f32>, timestamp_us: u64) {
        let mut retained = 0;
        for index in 0..self.matrix_filled {
            if timestamp_us.saturating_sub(self.sample_timestamps_us[index])
                <= self.max_sample_lifespan_us
            {
                if retained != index {
                    for column in 0..3 {
                        self.matrix[(retained, column)] = self.matrix[(index, column)];
                    }
                    self.sample_timestamps_us[retained] = self.sample_timestamps_us[index];
                }
                retained += 1;
            }
        }
        if retained != self.matrix_filled {
            self.matrix_filled = retained;
            self.mean_distance = 0.0;
        }

        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
            return;
        }
        // Check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            self.add_sample_at(self.matrix_filled, x, timestamp_us);
            self.matrix_filled += 1;
        }
        // Otherwise check which sample may be best to replace
        else {
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let sample_mean_dist = self.mean_distance_from_single(x.transpose());
            if low_mean_dist < sample_mean_dist {
                self.add_sample_at(low_index, x, timestamp_us);
            }
        }
    }

    /// Insert a sample vector into `index` row of buffer matrix.
    fn add_sample_at(&mut self, index: usize, sample: Vector3<f32>, timestamp_us: u64) {
        if index < N {
            self.matrix[(index, 0)] = sample[0];
            self.matrix[(index, 1)] = sample[1];
            self.matrix[(index, 2)] = sample[2];
            self.sample_timestamps_us[index] = timestamp_us;
        }
    }

    /// Get mean distance value between samples in matrix buffer.
    pub fn get_mean_distance(&self) -> f32 {
        self.mean_distance
    }

    /// Calibrates a magnetometer vector that has already been converted to FRD.
    pub fn evaluate_correct(
        &mut self,
        raw_mag: Vector3<f32>,
        timestamp_us: u64,
    ) -> Result<Vector3<f32>, BadMagCause> {
        self.evaluate_sample_vec(raw_mag, timestamp_us);
        // _REVIEW: this should return an Option<Unit>, if the new sample is not added, there is no need to perform_calibration, the reading can be corrected using previous state directly.
        // REBUTTAL: Re-running the warm-started solve is intentional: a capped BCD
        // result is persisted and refined even when the KNN buffer rejects the sample.
        self.perform_calibration()?;
        let mag = Self::corrected(
            raw_mag - self.hard_iron_offset,
            &self.inverse_soft_iron_cholesky,
        );

        let mag_norm = mag.norm();
        if !mag_norm.is_finite() || mag_norm < MIN_MAG_NORM {
            Err(BadMagCause::BadReading(BadReading::WeakCalibratedReading {
                norm: mag_norm,
                min_norm: MIN_MAG_NORM,
            }))
        } else {
            Ok(mag.normalize())
        }
    }

    /// Refines the saved hard-iron offset and inverse soft-iron factor with a
    /// fixed-budget regularized least-squares coordinate descent.
    ///
    /// On success, persists the updated calibration state. Returns the cause when
    /// there are not enough samples to start calibration.
    fn perform_calibration(&mut self) -> Result<(), BadCalibration> {
        let sample_count = self.matrix_filled.min(N);
        let required_samples = N.max(CALIBRATION_PARAMETER_COUNT);
        if sample_count < required_samples {
            return Err(BadCalibration::InsufficientSamples {
                samples: sample_count,
                required: required_samples,
            });
        }

        let sample_mean = (0..sample_count)
            .fold(Vector3::zeros(), |sum, row| sum + self.sample(row))
            / sample_count as f32;
        let first_calibration =
            !self.calibration_initialized && sample_mean.iter().all(|value| value.is_finite());
        let mut offset = if first_calibration {
            sample_mean
        } else {
            self.hard_iron_offset
        };
        let mut inverse_cholesky = self.inverse_soft_iron_cholesky;
        if first_calibration {
            let mean_squared_radius = (0..sample_count)
                .map(|row| (self.sample(row) - sample_mean).norm_squared())
                .sum::<f32>()
                / sample_count as f32;
            let factor_scale = mean_squared_radius.sqrt().sqrt();
            if factor_scale.is_finite() && factor_scale > f32::EPSILON {
                inverse_cholesky = Matrix3::identity() * factor_scale;
            }
        }

        for _ in 0..CALIBRATION_SWEEPS {
            self.update_offset(sample_count, &mut offset, &inverse_cholesky);
            self.update_inverse_cholesky(sample_count, &offset, &mut inverse_cholesky);
        }

        self.hard_iron_offset = offset;
        self.inverse_soft_iron_cholesky = inverse_cholesky;
        self.calibration_initialized = true;
        Ok(())
    }

    fn sample(&self, row: usize) -> Vector3<f32> {
        // TODO: this should be a linear algebra operation, avoid elementwise operations
        Vector3::new(
            self.matrix[(row, 0)],
            self.matrix[(row, 1)],
            self.matrix[(row, 2)],
        )
    }

    // TODO: wrong name: mag vector must be "corrected" by BOTH hard & soft iron distortion
    fn corrected(vector: Vector3<f32>, inverse_cholesky: &Matrix3<f32>) -> Vector3<f32> {
        // TODO: this should be a linear algebra operation, avoid elementwise operations
        let y0 = vector.x / inverse_cholesky[(0, 0)];
        let y1 = (vector.y - inverse_cholesky[(1, 0)] * y0) / inverse_cholesky[(1, 1)];
        let y2 = (vector.z - inverse_cholesky[(2, 0)] * y0 - inverse_cholesky[(2, 1)] * y1)
            / inverse_cholesky[(2, 2)];
        let x2 = y2 / inverse_cholesky[(2, 2)];
        let x1 = (y1 - inverse_cholesky[(2, 1)] * x2) / inverse_cholesky[(1, 1)];
        let x0 = (y0 - inverse_cholesky[(1, 0)] * x1 - inverse_cholesky[(2, 0)] * x2)
            / inverse_cholesky[(0, 0)];
        Vector3::new(x0, x1, x2)
    }

    fn update_offset(
        &self,
        sample_count: usize,
        offset: &mut Vector3<f32>,
        inverse_cholesky: &Matrix3<f32>,
    ) {
        let mut gradient = [0.0; 3];
        let mut curvature = [0.0; 3];

        for row in 0..sample_count {
            let corrected = Self::corrected(self.sample(row) - *offset, inverse_cholesky);
            let norm = corrected.norm();
            if !norm.is_finite() || norm <= f32::EPSILON {
                continue;
            }
            let residual = norm - 1.0;
            let jacobian = -Self::corrected(corrected, inverse_cholesky) / norm;
            for index in 0..3 {
                if jacobian[index].is_finite() {
                    gradient[index] += jacobian[index] * residual;
                    curvature[index] += jacobian[index] * jacobian[index];
                }
            }
        }

        for index in 0..3 {
            let delta = (gradient[index] / curvature[index].max(f32::EPSILON))
                .clamp(-MAX_PARAMETER_STEP, MAX_PARAMETER_STEP);
            let candidate = offset[index] - delta;
            if delta.is_finite() && candidate.is_finite() {
                offset[index] = candidate;
            }
        }
    }

    fn update_inverse_cholesky(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        inverse_cholesky: &mut Matrix3<f32>,
    ) {
        let parameter_indices = [(0, 0), (1, 0), (1, 1), (2, 0), (2, 1), (2, 2)];
        let mut gradient = [0.0; 6];
        let mut curvature = [0.0; 6];

        for row in 0..sample_count {
            let corrected = Self::corrected(self.sample(row) - offset, inverse_cholesky);
            let norm = corrected.norm();
            if !norm.is_finite() || norm <= f32::EPSILON {
                continue;
            }
            let residual = norm - 1.0;
            let corrected_again = Self::corrected(corrected, inverse_cholesky);
            let derivative = -(corrected_again
                * (inverse_cholesky.transpose() * corrected).transpose()
                + corrected * (inverse_cholesky.transpose() * corrected_again).transpose())
                / norm;
            for (index, &(matrix_row, matrix_column)) in parameter_indices.iter().enumerate() {
                let jacobian = derivative[(matrix_row, matrix_column)];
                if jacobian.is_finite() {
                    gradient[index] += jacobian * residual;
                    curvature[index] += jacobian * jacobian;
                }
            }
        }

        let regularization = FACTOR_REGULARIZATION * sample_count as f32;
        for (index, &(matrix_row, matrix_column)) in parameter_indices.iter().enumerate() {
            let value = inverse_cholesky[(matrix_row, matrix_column)];
            let delta = ((gradient[index] + regularization * value)
                / (curvature[index] + regularization))
                .clamp(-MAX_PARAMETER_STEP, MAX_PARAMETER_STEP);
            let candidate = value - delta;
            if delta.is_finite() && candidate.is_finite() {
                inverse_cholesky[(matrix_row, matrix_column)] = candidate;
            }
        }
    }
}
