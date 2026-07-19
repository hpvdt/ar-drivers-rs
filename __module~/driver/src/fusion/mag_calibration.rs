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
    soft_iron_correction_factor: Matrix3<f32>,
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
            soft_iron_correction_factor: Matrix3::identity(),
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
        let k = self.k.min(N.saturating_sub(1));
        if k == 0 {
            return f32::INFINITY;
        }

        let mut nearest_squared_distances = [f32::INFINITY; N];
        for row in 0..N {
            let x = vec[(0, 0)] - self.matrix[(row, 0)];
            let y = vec[(0, 1)] - self.matrix[(row, 1)];
            let z = vec[(0, 2)] - self.matrix[(row, 2)];
            let squared_distance = x * x + y * y + z * z;

            let mut insertion_index = 0;
            while insertion_index <= k
                && nearest_squared_distances[insertion_index] <= squared_distance
            {
                insertion_index += 1;
            }
            if insertion_index <= k {
                nearest_squared_distances.copy_within(insertion_index..k, insertion_index + 1);
                nearest_squared_distances[insertion_index] = squared_distance;
            }
        }

        nearest_squared_distances
            .iter()
            .skip(1)
            .take(k)
            .rfold(0., |sum, squared_distance| sum + squared_distance.sqrt())
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
        let mag = Self::apply_correction(
            raw_mag - self.hard_iron_offset,
            &self.soft_iron_correction_factor,
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

    /// Refines the saved hard-iron offset and soft-iron correction factor with a
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
        let mut correction_factor = self.soft_iron_correction_factor;
        if first_calibration {
            let mean_squared_radius = (0..sample_count)
                .map(|row| (self.sample(row) - sample_mean).norm_squared())
                .sum::<f32>()
                / sample_count as f32;
            let rho = mean_squared_radius.sqrt().sqrt();
            if rho.is_finite() && rho > f32::EPSILON {
                correction_factor = Matrix3::identity() * rho.recip();
            }
        }
        let offset_step_relaxation = if first_calibration {
            3.0
        } else {
            sample_count as f32
        };
        let factor_step_relaxation = if first_calibration {
            6.0
        } else {
            sample_count as f32
        };

        for _ in 0..CALIBRATION_SWEEPS {
            self.update_offset(
                sample_count,
                &mut offset,
                &correction_factor,
                offset_step_relaxation,
            );
            self.update_correction_factor(
                sample_count,
                &offset,
                &mut correction_factor,
                factor_step_relaxation,
            );
        }

        self.hard_iron_offset = offset;
        self.soft_iron_correction_factor = correction_factor;
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

    #[inline(always)]
    fn apply_factor(vector: Vector3<f32>, factor: &Matrix3<f32>) -> Vector3<f32> {
        Vector3::new(
            factor[(0, 0)] * vector.x,
            factor[(1, 0)] * vector.x + factor[(1, 1)] * vector.y,
            factor[(2, 0)] * vector.x + factor[(2, 1)] * vector.y + factor[(2, 2)] * vector.z,
        )
    }

    #[inline(always)]
    fn apply_factor_transpose(vector: Vector3<f32>, factor: &Matrix3<f32>) -> Vector3<f32> {
        Vector3::new(
            factor[(0, 0)] * vector.x + factor[(1, 0)] * vector.y + factor[(2, 0)] * vector.z,
            factor[(1, 1)] * vector.y + factor[(2, 1)] * vector.z,
            factor[(2, 2)] * vector.z,
        )
    }

    #[inline(always)]
    fn apply_correction(vector: Vector3<f32>, correction_factor: &Matrix3<f32>) -> Vector3<f32> {
        Self::apply_factor_transpose(
            Self::apply_factor(vector, correction_factor),
            correction_factor,
        )
    }

    fn update_offset(
        &self,
        sample_count: usize,
        offset: &mut Vector3<f32>,
        correction_factor: &Matrix3<f32>,
        step_relaxation: f32,
    ) {
        let mut gradient = [0.0; 3];
        let mut curvature = [0.0; 3];

        for row in 0..sample_count {
            let corrected = Self::apply_correction(self.sample(row) - *offset, correction_factor);
            let norm = corrected.norm();
            if !norm.is_finite() || norm <= f32::EPSILON {
                continue;
            }
            let residual = norm - 1.0;
            let jacobian = -Self::apply_correction(corrected, correction_factor) / norm;
            for index in 0..3 {
                let gradient_contribution = jacobian[index] * residual;
                let curvature_contribution = jacobian[index] * jacobian[index];
                if gradient_contribution.is_finite() && curvature_contribution.is_finite() {
                    gradient[index] += gradient_contribution;
                    curvature[index] += curvature_contribution;
                }
            }
        }

        for index in 0..3 {
            let delta = (gradient[index] / curvature[index].max(f32::EPSILON) / step_relaxation)
                .clamp(-MAX_PARAMETER_STEP, MAX_PARAMETER_STEP);
            let candidate = offset[index] - delta;
            if delta.is_finite() && candidate.is_finite() {
                offset[index] = candidate;
            }
        }
    }

    fn update_correction_factor(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        correction_factor: &mut Matrix3<f32>,
        step_relaxation: f32,
    ) {
        let parameter_indices = [(0, 0), (1, 0), (1, 1), (2, 0), (2, 1), (2, 2)];
        let mut gradient = [0.0; 6];
        let mut curvature = [0.0; 6];

        for row in 0..sample_count {
            let centered = self.sample(row) - offset;
            let projected = Self::apply_factor(centered, correction_factor);
            let corrected = Self::apply_factor_transpose(projected, correction_factor);
            let norm = corrected.norm();
            if !norm.is_finite() || norm <= f32::EPSILON {
                continue;
            }
            let residual = norm - 1.0;
            let factor_corrected = Self::apply_factor(corrected, correction_factor);
            for (index, &(matrix_row, matrix_column)) in parameter_indices.iter().enumerate() {
                let jacobian = (corrected[matrix_column] * projected[matrix_row]
                    + centered[matrix_column] * factor_corrected[matrix_row])
                    / norm;
                let gradient_contribution = jacobian * residual;
                let curvature_contribution = jacobian * jacobian;
                if gradient_contribution.is_finite() && curvature_contribution.is_finite() {
                    gradient[index] += gradient_contribution;
                    curvature[index] += curvature_contribution;
                }
            }
        }

        let regularization = FACTOR_REGULARIZATION * sample_count as f32;
        for (index, &(matrix_row, matrix_column)) in parameter_indices.iter().enumerate() {
            let value = correction_factor[(matrix_row, matrix_column)];
            let delta = ((gradient[index] + regularization * value)
                / (curvature[index] + regularization)
                / step_relaxation)
                .clamp(-MAX_PARAMETER_STEP, MAX_PARAMETER_STEP);
            let candidate = value - delta;
            if delta.is_finite() && candidate.is_finite() {
                correction_factor[(matrix_row, matrix_column)] = candidate;
            }
        }
    }
}
