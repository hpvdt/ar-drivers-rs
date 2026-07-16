// use core::cmp::Ordering;
use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SMatrixView, SVector, Vector3, SVD};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const DESIGN_MATRIX_COLUMNS: usize = 9;
const SVD_EPSILON_RATIO: f32 = 1.0e-6;
const MAX_SVD_CONDITION: f32 = 1.0e6;
const MIN_MAG_NORM: f32 = 0.4;

/// Lightweight least squares approach to
/// determining the offset and scaling
/// factors for magnetometer calibration.
/// Also includes the capability to automatically
/// collect good data points, using a `const`-sized
/// buffer matrix, and a k-nearest neighbors.
///
/// source: https://github.com/peterkrull/mag-calibrator-rs/blob/main/src/lib.rs
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 6>,
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    // REVIEW: previous state of hard/soft-iron should be initialised & persisted here.
    mean_distance: f32,
    pre_scaler: f32,
    k: usize,
    max_sample_lifespan_us: u64,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::from_element(1.0),
            sample_timestamps_us: [0; N],
            matrix_filled: Default::default(),
            mean_distance: Default::default(),
            pre_scaler: 1.,
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

    /// Configure sample pre scaler, prevents ill-conditioning if given
    /// a value close to the expected magnitude of the magnetic field strength.
    /// This is an internal numerical aid only: it scales samples before the
    /// least-squares fit and is undone before returning, so the offset and scale
    /// from `perform_calibration` are always in the same units as the raw samples
    /// and are unaffected by this value.
    pub fn pre_scaler(self, pre_scaler: f32) -> Self {
        Self { pre_scaler, ..self }
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
            self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
            self.matrix[(index, 1)] = sample[1] / self.pre_scaler;
            self.matrix[(index, 2)] = sample[2] / self.pre_scaler;
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
        let (offset, correction) = self.perform_calibration()?;
        let mag = correction * (raw_mag - offset);

        let mag_norm = mag.norm();
        if mag_norm < MIN_MAG_NORM {
            Err(BadMagCause::BadReading(BadReading::WeakCalibratedReading {
                norm: mag_norm,
                min_norm: MIN_MAG_NORM,
            }))
        } else {
            Ok(mag.normalize())
        }
    }

    /// Try to calculate the hard-iron offset and full SPD soft-iron correction.
    /// Returns the cause when the calibration cannot be produced.
    pub fn perform_calibration(&mut self) -> Result<(Vector3<f32>, Matrix3<f32>), BadCalibration> {
        let sample_count = self.matrix_filled.min(N);
        let required_samples = N.max(DESIGN_MATRIX_COLUMNS);
        if sample_count < required_samples {
            return Err(BadCalibration::InsufficientSamples {
                samples: sample_count,
                required: required_samples,
            });
        }

        let sample_mean = (0..sample_count).fold(Vector3::zeros(), |sum, row| {
            sum + Vector3::new(
                self.matrix[(row, 0)],
                self.matrix[(row, 1)],
                self.matrix[(row, 2)],
            )
        }) / sample_count as f32;
        let sample_scale = ((0..sample_count).fold(0.0, |sum, row| {
            let sample = Vector3::new(
                self.matrix[(row, 0)],
                self.matrix[(row, 1)],
                self.matrix[(row, 2)],
            );
            sum + (sample - sample_mean).norm_squared()
        }) / sample_count as f32)
            .sqrt();
        if !sample_scale.is_finite() || sample_scale <= f32::EPSILON {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: f32::INFINITY,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        let samples = DMatrix::from_fn(sample_count, 3, |row, col| self.matrix[(row, col)]);
        let design = DMatrix::from_fn(sample_count, DESIGN_MATRIX_COLUMNS, |row, col| {
            let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
            let sample = (sample - sample_mean) / sample_scale;
            match col {
                0 => sample.x * sample.x,
                1 => sample.y * sample.y,
                2 => sample.z * sample.z,
                3 => 2.0 * sample.x * sample.y,
                4 => 2.0 * sample.x * sample.z,
                5 => 2.0 * sample.y * sample.z,
                6 => sample.x,
                7 => sample.y,
                8 => sample.z,
                _ => unreachable!(),
            }
        });
        let w = DVector::from_element(sample_count, 1.0); // REVIEW: use full name, not symbol in equation

        let svd = SVD::new(design, true, true);
        // REVIEW: SVD should be completely superseded by Cholesky-factor based full soft-iron matrix estimation using alternating block-coordinate descent: starting from the saved state of hard-iron cholesky factor & soft-iron bias. As a result, SVD reference and anything that depends on it should be deleted.
        let singular_values = svd.singular_values.as_slice();
        let max_singular_value = singular_values[0];
        let min_singular_value = singular_values[DESIGN_MATRIX_COLUMNS - 1];
        let epsilon = max_singular_value * SVD_EPSILON_RATIO;

        let condition = max_singular_value / min_singular_value;
        if !condition.is_finite() || condition > MAX_SVD_CONDITION {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        // Solve the least-squares system with a Moore-Penrose pseudo-inverse.
        let pseudo_inverse = svd
            .pseudo_inverse(epsilon)
            .map_err(|message| BadCalibration::Unsolveable { message })?;
        let x = pseudo_inverse * w;
        let shape = Matrix3::new(x[0], x[3], x[4], x[3], x[1], x[5], x[4], x[5], x[2]);
        let linear = Vector3::new(x[6], x[7], x[8]);
        let shape_inverse =
            shape
                .try_inverse()
                .ok_or(BadCalibration::DegenerateSoftIronMatrix {
                    condition: f32::INFINITY,
                    max_condition: MAX_SVD_CONDITION,
                })?;
        let normalized_offset = -0.5 * shape_inverse * linear;
        let radius_squared = 1.0 + normalized_offset.dot(&(shape * normalized_offset));
        if !radius_squared.is_finite() || radius_squared <= f32::EPSILON {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: f32::INFINITY,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        let normalized_shape = shape / radius_squared;
        let eigen = normalized_shape.symmetric_eigen();
        let min_eigenvalue = eigen.eigenvalues.min();
        let max_eigenvalue = eigen.eigenvalues.max();
        let shape_condition = max_eigenvalue / min_eigenvalue;
        if !shape_condition.is_finite()
            || min_eigenvalue <= f32::EPSILON
            || shape_condition > MAX_SVD_CONDITION
        {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: shape_condition,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        let correction_normalized = eigen.eigenvectors
            * Matrix3::from_diagonal(&eigen.eigenvalues.map(f32::sqrt))
            * eigen.eigenvectors.transpose();
        let mut offset = sample_mean + normalized_offset * sample_scale;
        let correction = correction_normalized / sample_scale;
        let mut cholesky = correction
            .cholesky()
            .ok_or(BadCalibration::DegenerateSoftIronMatrix {
                condition: shape_condition,
                max_condition: MAX_SVD_CONDITION,
            })?
            .l();

        let robust_objective = |candidate_offset: &Vector3<f32>, candidate_l: &Matrix3<f32>| {
            let candidate_correction = candidate_l * candidate_l.transpose();
            let mut objective = 0.0;
            for row in 0..sample_count {
                let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
                let residual = (candidate_correction * (sample - candidate_offset)).norm() - 1.0;
                let absolute = residual.abs();
                objective += if absolute <= 0.1 {
                    0.5 * residual * residual
                } else {
                    0.1 * (absolute - 0.05)
                };
            }
            objective / sample_count as f32
        };
        let mut objective = robust_objective(&offset, &cholesky);
        let mut converged = false;
        for _ in 0..50 {
            let offset_updated = self.update_offset(&samples, &mut offset, &cholesky);
            let cholesky_updated = self.update_cholesky(&samples, &offset, &mut cholesky);
            let next_objective = robust_objective(&offset, &cholesky);
            let improvement = objective - next_objective;
            if (!offset_updated && !cholesky_updated) || improvement <= 1.0e-5 * objective.max(1.0)
            {
                converged = true;
                objective = next_objective;
                break;
            }
            objective = next_objective;
        }
        if !converged {
            return Err(BadCalibration::Unsolveable {
                message: "full soft-iron calibration did not converge",
            });
        }

        let correction = cholesky * cholesky.transpose();
        let radial_rms = ((0..sample_count).fold(0.0, |sum, row| {
            let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
            let residual = (correction * (sample - offset)).norm() - 1.0;
            sum + residual * residual
        }) / sample_count as f32)
            .sqrt();
        let eigenvalues = correction.symmetric_eigenvalues();
        let correction_condition = eigenvalues.max() / eigenvalues.min();
        if !objective.is_finite()
            || !radial_rms.is_finite()
            || radial_rms > 0.15
            || !correction_condition.is_finite()
            || eigenvalues.min() <= f32::EPSILON
            || correction_condition > MAX_SVD_CONDITION
        {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: correction_condition,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        // Samples are divided by `pre_scaler` before storage, so convert the
        // fitted offset and correction back to caller-visible raw units.
        offset *= self.pre_scaler;
        let correction = correction / self.pre_scaler;
        if !offset
            .iter()
            .chain(correction.iter())
            .all(|value| value.is_finite())
        {
            return Err(BadCalibration::Unsolveable {
                message: "full soft-iron calibration produced non-finite parameters",
            });
        }

        // TODO Add option for low-pass filtering this result
        Ok((offset, correction))
    }

    fn update_offset(
        &self,
        samples: &DMatrix<f32>,
        offset: &mut Vector3<f32>,
        cholesky: &Matrix3<f32>,
    ) -> bool {
        let correction = cholesky * cholesky.transpose();
        let objective = |candidate: &Vector3<f32>| {
            let mut value = 0.0;
            for row in 0..samples.nrows() {
                let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
                let residual = (correction * (sample - candidate)).norm() - 1.0;
                let absolute = residual.abs();
                value += if absolute <= 0.1 {
                    0.5 * residual * residual
                } else {
                    0.1 * (absolute - 0.05)
                };
            }
            value / samples.nrows() as f32
        };
        let current_objective = objective(offset);
        let mut hessian = Matrix3::zeros();
        let mut gradient = Vector3::zeros();
        for row in 0..samples.nrows() {
            let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
            let centered = sample - *offset;
            let corrected = correction * centered;
            let norm = corrected.norm();
            if norm <= f32::EPSILON {
                return false;
            }
            let residual = norm - 1.0;
            let weight = if residual.abs() <= 0.1 {
                1.0
            } else {
                0.1 / residual.abs()
            };
            let jacobian = -(correction.transpose() * corrected) / norm;
            hessian += weight * jacobian * jacobian.transpose();
            gradient += weight * jacobian * residual;
        }

        for attempt in 0..6 {
            let damping = 1.0e-4 * 10.0f32.powi(attempt);
            let Some(inverse) = (hessian + Matrix3::identity() * damping).try_inverse() else {
                continue;
            };
            let candidate = *offset - inverse * gradient;
            let candidate_objective = objective(&candidate);
            if candidate_objective.is_finite() && candidate_objective < current_objective {
                *offset = candidate;
                return true;
            }
        }
        false
    }

    fn update_cholesky(
        &self,
        samples: &DMatrix<f32>,
        offset: &Vector3<f32>,
        cholesky: &mut Matrix3<f32>,
    ) -> bool {
        let objective = |candidate_l: &Matrix3<f32>| {
            let candidate_correction = candidate_l * candidate_l.transpose();
            let mut value = 0.0;
            for row in 0..samples.nrows() {
                let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
                let residual = (candidate_correction * (sample - offset)).norm() - 1.0;
                let absolute = residual.abs();
                value += if absolute <= 0.1 {
                    0.5 * residual * residual
                } else {
                    0.1 * (absolute - 0.05)
                };
            }
            value / samples.nrows() as f32
        };
        let current_objective = objective(cholesky);
        let correction = *cholesky * cholesky.transpose();
        let mut hessian = SMatrix::<f32, 6, 6>::zeros();
        let mut gradient = SVector::<f32, 6>::zeros();
        for row in 0..samples.nrows() {
            let sample = Vector3::new(samples[(row, 0)], samples[(row, 1)], samples[(row, 2)]);
            let centered = sample - offset;
            let corrected = correction * centered;
            let norm = corrected.norm();
            if norm <= f32::EPSILON {
                return false;
            }
            let residual = norm - 1.0;
            let weight = if residual.abs() <= 0.1 {
                1.0
            } else {
                0.1 / residual.abs()
            };
            let mut jacobian = SVector::<f32, 6>::zeros();
            for parameter in 0..6 {
                let mut derivative_l = Matrix3::zeros();
                match parameter {
                    0 => derivative_l[(0, 0)] = cholesky[(0, 0)],
                    1 => derivative_l[(1, 0)] = 1.0,
                    2 => derivative_l[(1, 1)] = cholesky[(1, 1)],
                    3 => derivative_l[(2, 0)] = 1.0,
                    4 => derivative_l[(2, 1)] = 1.0,
                    5 => derivative_l[(2, 2)] = cholesky[(2, 2)],
                    _ => unreachable!(),
                }
                let derivative_correction =
                    derivative_l * cholesky.transpose() + *cholesky * derivative_l.transpose();
                jacobian[parameter] = corrected.dot(&(derivative_correction * centered)) / norm;
            }
            hessian += weight * jacobian * jacobian.transpose();
            gradient += weight * jacobian * residual;
        }

        let parameters = SVector::<f32, 6>::new(
            cholesky[(0, 0)].ln(),
            cholesky[(1, 0)],
            cholesky[(1, 1)].ln(),
            cholesky[(2, 0)],
            cholesky[(2, 1)],
            cholesky[(2, 2)].ln(),
        );
        for attempt in 0..6 {
            let damping = 1.0e-4 * 10.0f32.powi(attempt);
            let Some(inverse) =
                (hessian + SMatrix::<f32, 6, 6>::identity() * damping).try_inverse()
            else {
                continue;
            };
            let candidate_parameters = parameters - inverse * gradient;
            let candidate = Matrix3::new(
                candidate_parameters[0].exp(),
                0.0,
                0.0,
                candidate_parameters[1],
                candidate_parameters[2].exp(),
                0.0,
                candidate_parameters[3],
                candidate_parameters[4],
                candidate_parameters[5].exp(),
            );
            let candidate_objective = objective(&candidate);
            if candidate_objective.is_finite() && candidate_objective < current_objective {
                *cholesky = candidate;
                return true;
            }
        }
        false
    }
}
