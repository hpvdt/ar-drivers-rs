use nalgebra::{Matrix3, SMatrix, SMatrixView, SVector, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
const SHAPE_REGULARIZATION: f32 = 1.0e-4;
const MAX_SAMPLE_CONDITION: f32 = 1.0e6;
const MAX_CORRECTION_CONDITION: f32 = 1.0e3;
const MAX_RADIAL_RMS: f32 = 0.25;
const MIN_MAG_NORM: f32 = 0.4;

/// Direct regularized ellipsoid fit for a hard-iron offset and full SPD
/// soft-iron correction from a fixed, diverse sample buffer.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 3>,
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    hard_iron_offset: Vector3<f32>,
    soft_iron_correction: Matrix3<f32>,
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
            soft_iron_correction: Matrix3::identity(),
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
        if let Err(error) = self.perform_calibration() {
            if !self.calibration_initialized
                || matches!(error, BadCalibration::InsufficientSamples { .. })
            {
                return Err(error.into());
            }
        }
        let mag = self.soft_iron_correction * (raw_mag - self.hard_iron_offset);

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

    /// Fits a regularized quadratic ellipsoid and derives its symmetric
    /// correction matrix.
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
        if !sample_mean.iter().all(|value| value.is_finite()) {
            return Err(BadCalibration::Unsolveable {
                message: "sample mean is non-finite",
            });
        }

        let covariance = (0..sample_count).fold(Matrix3::zeros(), |sum, row| {
            let centered = self.sample(row) - sample_mean;
            sum + centered * centered.transpose()
        }) / sample_count as f32;
        let radius_squared = covariance.trace();
        if !radius_squared.is_finite() || radius_squared <= f32::EPSILON {
            return Err(BadCalibration::Unsolveable {
                message: "sample radius is non-finite or zero",
            });
        }

        let sample_condition = Self::condition_number(&covariance.symmetric_eigen().eigenvalues);
        if sample_condition > MAX_SAMPLE_CONDITION {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: sample_condition,
                max_condition: MAX_SAMPLE_CONDITION,
            });
        }

        let radius = radius_squared.sqrt();
        let mut normal = SMatrix::<f32, 9, 9>::zeros();
        let mut right_hand_side = SVector::<f32, 9>::zeros();
        for row in 0..sample_count {
            let sample = (self.sample(row) - sample_mean) / radius;
            let features = SVector::<f32, 9>::from_row_slice(&[
                sample.x * sample.x,
                sample.y * sample.y,
                sample.z * sample.z,
                2.0 * sample.x * sample.y,
                2.0 * sample.x * sample.z,
                2.0 * sample.y * sample.z,
                sample.x,
                sample.y,
                sample.z,
            ]);
            normal += features * features.transpose();
            right_hand_side += features;
        }
        normal /= sample_count as f32;
        right_hand_side /= sample_count as f32;
        for (index, weight) in [1.0, 1.0, 1.0, 2.0, 2.0, 2.0].into_iter().enumerate() {
            normal[(index, index)] += SHAPE_REGULARIZATION * weight;
        }

        let parameters = normal
            .cholesky()
            .ok_or(BadCalibration::Unsolveable {
                message: "quadratic calibration system is singular",
            })?
            .solve(&right_hand_side);
        if !parameters.iter().all(|value| value.is_finite()) {
            return Err(BadCalibration::Unsolveable {
                message: "quadratic calibration produced non-finite parameters",
            });
        }

        let shape = Matrix3::new(
            parameters[0],
            parameters[3],
            parameters[4],
            parameters[3],
            parameters[1],
            parameters[5],
            parameters[4],
            parameters[5],
            parameters[2],
        );
        let linear = Vector3::new(parameters[6], parameters[7], parameters[8]);
        let shape_eigen = shape.symmetric_eigen();
        let correction_condition = Self::condition_number(&shape_eigen.eigenvalues).sqrt();
        if correction_condition > MAX_CORRECTION_CONDITION {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: correction_condition,
                max_condition: MAX_CORRECTION_CONDITION,
            });
        }

        let shape_cholesky = shape
            .cholesky()
            .ok_or(BadCalibration::DegenerateSoftIronMatrix {
                condition: f32::INFINITY,
                max_condition: MAX_CORRECTION_CONDITION,
            })?;
        let normalized_offset = -0.5 * shape_cholesky.solve(&linear);
        let ellipsoid_scale = 1.0 + normalized_offset.dot(&(shape * normalized_offset));
        if !ellipsoid_scale.is_finite() || ellipsoid_scale <= f32::EPSILON {
            return Err(BadCalibration::Unsolveable {
                message: "ellipsoid normalization is non-positive",
            });
        }

        let square_root = Matrix3::from_diagonal(
            &shape_eigen
                .eigenvalues
                .map(|value| (value / ellipsoid_scale).sqrt()),
        );
        let correction =
            shape_eigen.eigenvectors * square_root * shape_eigen.eigenvectors.transpose() / radius;
        let offset = sample_mean + radius * normalized_offset;
        if !offset.iter().all(|value| value.is_finite())
            || !correction.iter().all(|value| value.is_finite())
        {
            return Err(BadCalibration::Unsolveable {
                message: "calibration produced non-finite parameters",
            });
        }

        let radial_rms = ((0..sample_count)
            .map(|row| {
                let residual = (correction * (self.sample(row) - offset)).norm() - 1.0;
                residual * residual
            })
            .sum::<f32>()
            / sample_count as f32)
            .sqrt();
        if !radial_rms.is_finite() || radial_rms > MAX_RADIAL_RMS {
            return Err(BadCalibration::Unsolveable {
                message: "calibration radial residual is too large",
            });
        }

        self.hard_iron_offset = offset;
        self.soft_iron_correction = correction;
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

    fn condition_number(eigenvalues: &Vector3<f32>) -> f32 {
        let min = eigenvalues.iter().copied().fold(f32::INFINITY, f32::min);
        let max = eigenvalues
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max);
        if !min.is_finite() || !max.is_finite() || min <= 0.0 {
            f32::INFINITY
        } else {
            max / min
        }
    }
}
