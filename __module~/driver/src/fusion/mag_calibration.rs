use nalgebra::{Matrix3, SMatrix, SMatrixView, SVector, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
const MAX_MATRIX_CONDITION: f32 = 1.0e6;
const MIN_MAG_NORM: f32 = 0.4;

/// Alternating block-coordinate descent for estimating a hard-iron offset and
/// a full SPD soft-iron correction from a fixed, diverse sample buffer.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 3>,
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    hard_iron_offset: Vector3<f32>,
    soft_iron_cholesky: Matrix3<f32>,
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
            soft_iron_cholesky: Matrix3::identity(),
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
        // TODO: this should return an Option<Unit>, if the new sample is not added, there is no need to perform_calibration, the reading can be corrected using previous state directly.
        // REBUTTAL: Re-running the warm-started solve is intentional: a capped BCD
        // result is persisted and refined even when the KNN buffer rejects the sample.
        self.perform_calibration()?;
        let correction = self.soft_iron_cholesky * self.soft_iron_cholesky.transpose();
        let mag = correction * (raw_mag - self.hard_iron_offset);

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

    /// Refines the saved hard-iron offset and full SPD soft-iron calibration by
    /// alternating block-coordinate descent.
    ///
    /// On success, persists and returns `(hard_iron_offset, soft_iron_cholesky)`.
    /// The Cholesky factor has a positive diagonal and represents the correction
    /// matrix as `soft_iron_cholesky * soft_iron_cholesky.transpose()`.
    /// Returns the cause when the calibration cannot be produced.
    pub fn perform_calibration(&mut self) -> Result<(Vector3<f32>, Matrix3<f32>), BadCalibration> {
        let sample_count = self.matrix_filled.min(N);
        let required_samples = N.max(CALIBRATION_PARAMETER_COUNT);
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
        let sample_covariance = (0..sample_count).fold(Matrix3::zeros(), |sum, row| {
            let sample = Vector3::new(
                self.matrix[(row, 0)],
                self.matrix[(row, 1)],
                self.matrix[(row, 2)],
            );
            let centered = sample - sample_mean;
            sum + centered * centered.transpose()
        }) / sample_count as f32;
        // REVIEW: cholesky decomposition is expensive and unnecessary, particularly when cholesky factors are already available in the saved state or the alternating block descent
        // REBUTTAL: This factor describes the current sample covariance, whereas the
        // saved soft-iron factor describes the previous calibration. A single 3x3
        // factorization rejects degenerate buffers before the iterative solve.
        let sample_coverage_cholesky =
            sample_covariance
                .cholesky()
                .ok_or(BadCalibration::DegenerateSoftIronMatrix {
                    condition: f32::INFINITY,
                    max_condition: MAX_MATRIX_CONDITION,
                })?;
        let sample_coverage_factor = sample_coverage_cholesky.l();
        let min_coverage_diagonal = sample_coverage_factor[(0, 0)]
            .min(sample_coverage_factor[(1, 1)])
            .min(sample_coverage_factor[(2, 2)]);
        let max_coverage_diagonal = sample_coverage_factor[(0, 0)]
            .max(sample_coverage_factor[(1, 1)])
            .max(sample_coverage_factor[(2, 2)]);
        let sample_coverage_condition = (max_coverage_diagonal / min_coverage_diagonal).powi(2);
        if !sample_coverage_condition.is_finite()
            || sample_coverage_condition > MAX_MATRIX_CONDITION
        {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: sample_coverage_condition,
                max_condition: MAX_MATRIX_CONDITION,
            });
        }

        let mut offset = self.hard_iron_offset;
        let mut cholesky = self.soft_iron_cholesky;

        let mut correction = cholesky * cholesky.transpose();
        let mut objective = self.robust_radial_objective(sample_count, &offset, &correction);
        if !objective.is_finite() {
            return Err(BadCalibration::Unsolveable {
                message: "saved full soft-iron calibration state is not finite",
            });
        }
        for iteration in 0..200 {
            let offset_updated = self.update_offset(sample_count, &mut offset, &cholesky);
            let cholesky_updated = self.update_cholesky(sample_count, &offset, &mut cholesky);
            correction = cholesky * cholesky.transpose();
            let next_objective = self.robust_radial_objective(sample_count, &offset, &correction);
            let improvement = objective - next_objective;
            if (!offset_updated && !cholesky_updated)
                || (iteration >= 20 && improvement <= 1.0e-6 * objective.max(1.0))
            {
                objective = next_objective;
                break;
            }
            objective = next_objective;
        }

        // REVIEW: everything below are for computing corrected reading, not calibration.
        // REBUTTAL: These checks validate the candidate calibration before it is
        // persisted; corrected readings are computed only by `evaluate_correct`.
        let radial_rms = ((0..sample_count).fold(0.0, |sum, row| {
            let sample = self.sample(row);
            let residual = (correction * (sample - offset)).norm() - 1.0;
            sum + residual * residual
        }) / sample_count as f32)
            .sqrt();
        let correction_inverse =
            correction
                .try_inverse()
                .ok_or(BadCalibration::DegenerateSoftIronMatrix {
                    condition: f32::INFINITY,
                    max_condition: MAX_MATRIX_CONDITION,
                })?;
        let correction_condition = correction.norm() * correction_inverse.norm();
        if !objective.is_finite()
            || !radial_rms.is_finite()
            || radial_rms > 0.15
            || !correction_condition.is_finite()
            || !cholesky.iter().all(|value| value.is_finite())
            || cholesky[(0, 0)] <= f32::EPSILON
            || cholesky[(1, 1)] <= f32::EPSILON
            || cholesky[(2, 2)] <= f32::EPSILON
            || correction_condition > MAX_MATRIX_CONDITION
        {
            return Err(BadCalibration::DegenerateSoftIronMatrix {
                condition: correction_condition,
                max_condition: MAX_MATRIX_CONDITION,
            });
        }

        if !offset
            .iter()
            .chain(cholesky.iter())
            .all(|value| value.is_finite())
        {
            return Err(BadCalibration::Unsolveable {
                message: "full soft-iron calibration produced non-finite parameters",
            });
        }

        self.hard_iron_offset = offset;
        self.soft_iron_cholesky = cholesky;

        // TODO Add option for low-pass filtering this result
        Ok((offset, cholesky))
    }

    fn sample(&self, row: usize) -> Vector3<f32> {
        Vector3::new(
            self.matrix[(row, 0)],
            self.matrix[(row, 1)],
            self.matrix[(row, 2)],
        )
    }

    fn robust_radial_objective(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        correction: &Matrix3<f32>,
    ) -> f32 {
        (0..sample_count)
            .map(|row| (correction * (self.sample(row) - offset)).norm() - 1.0)
            .map(|residual| match residual.abs() {
                absolute if absolute <= 0.1 => 0.5 * residual * residual,
                absolute => 0.1 * (absolute - 0.05),
            })
            .sum::<f32>()
            / sample_count as f32
    }

    fn update_offset(
        &self,
        sample_count: usize,
        offset: &mut Vector3<f32>,
        cholesky: &Matrix3<f32>,
    ) -> bool {
        let correction = cholesky * cholesky.transpose();
        let current_objective = self.robust_radial_objective(sample_count, offset, &correction);
        let sample_mean = (0..sample_count)
            .fold(Vector3::zeros(), |sum, row| sum + self.sample(row))
            / sample_count as f32;
        if self.robust_radial_objective(sample_count, &sample_mean, &correction) < current_objective
        {
            *offset = sample_mean;
            return true;
        }
        let mut hessian = Matrix3::zeros();
        let mut gradient = Vector3::zeros();
        for row in 0..sample_count {
            let sample = self.sample(row);
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

        // Six damping decades let a rejected Gauss-Newton step fall back to a
        // conservative objective-decreasing step without an unbounded search.
        for damping_exponent in -4..=1 {
            let damping = 10.0f32.powi(damping_exponent);
            let Some(inverse) = (hessian + Matrix3::identity() * damping).try_inverse() else {
                continue;
            };
            let candidate = *offset - inverse * gradient;
            let candidate_objective =
                self.robust_radial_objective(sample_count, &candidate, &correction);
            if candidate_objective.is_finite() && candidate_objective < current_objective {
                *offset = candidate;
                return true;
            }
        }
        false
    }

    fn update_cholesky(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        cholesky: &mut Matrix3<f32>,
    ) -> bool {
        let correction = *cholesky * cholesky.transpose();
        let current_objective = self.robust_radial_objective(sample_count, offset, &correction);
        let mut hessian = SMatrix::<f32, 6, 6>::zeros();
        let mut gradient = SVector::<f32, 6>::zeros();
        for row in 0..sample_count {
            let sample = self.sample(row);
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
        // REVIEW: why do you need 6 attempts?
        // REBUTTAL: Six damping decades let a rejected Gauss-Newton step fall back
        // to a conservative objective-decreasing step without an unbounded search.
        for damping_exponent in -4..=1 {
            let damping = 10.0f32.powi(damping_exponent);
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
            let candidate_correction = candidate * candidate.transpose();
            let candidate_objective =
                self.robust_radial_objective(sample_count, offset, &candidate_correction);
            if candidate_objective.is_finite() && candidate_objective < current_objective {
                *cholesky = candidate;
                return true;
            }
        }
        false
    }
}
