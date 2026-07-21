use nalgebra::{Matrix3, SMatrix, SMatrixView, SVector, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
const CALIBRATION_SAMPLE_INTERVAL: usize = 4;
const CALIBRATION_SWEEPS: usize = 20;
const ALGEBRAIC_FIT_DAMPING: f32 = 1.0e-4;
const ALGEBRAIC_UPDATE_GAIN: f32 = 0.25;
const FACTOR_DAMPING: f32 = 1.0e-4;
const MAX_ALGEBRAIC_NORMAL_CONDITION: f32 = 1.0e6;
const MAX_NONLINEAR_CENTER_DISTANCE_RATIO: f32 = 0.95;
const MAX_NONLINEAR_TO_ALGEBRAIC_LOSS_RATIO: f32 = 2.0;
const MAX_SAMPLE_COVARIANCE_CONDITION: f32 = 1_000.0;
const MAX_PARAMETER_STEP: f32 = 0.25;
const MIN_MAG_NORM: f32 = 0.4;
const MIN_BCD_MAJOR_TO_MIDDLE_COVARIANCE_RATIO: f32 = 3.0 / 2.0;
const MIN_BCD_MINOR_TO_MIDDLE_COVARIANCE_RATIO: f32 = 1.0 / 3.0;
const NONLINEAR_ENSEMBLE_WEIGHT: f32 = 0.5;
const OBJECTIVE_ENSEMBLE_EXIT_LOSS_RATIO: f32 = 1.05;
const OBJECTIVE_ENSEMBLE_UPDATE_GAIN: f32 = 0.05;

/// Alternating block-coordinate descent for estimating a hard-iron offset and
/// a full SPD soft-iron correction from a fixed, diverse sample buffer.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 3>,
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    hard_iron_offset: Vector3<f32>,
    soft_iron_correction_factor: Matrix3<f32>,
    nonlinear_hard_iron_offset: Vector3<f32>,
    nonlinear_soft_iron_correction_factor: Matrix3<f32>,
    calibration_initialized: bool,
    nonlinear_calibration_initialized: bool,
    using_nonlinear_fit: bool,
    using_objective_ensemble: bool,
    stabilizing_objective_ensemble: bool,
    pending_sample_changes: usize,
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
            nonlinear_hard_iron_offset: Vector3::zeros(),
            nonlinear_soft_iron_correction_factor: Matrix3::identity(),
            calibration_initialized: false,
            nonlinear_calibration_initialized: false,
            using_nonlinear_fit: false,
            using_objective_ensemble: false,
            stabilizing_objective_ensemble: false,
            pending_sample_changes: 0,
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
        self.evaluate_sample_vec_changed(x, timestamp_us);
    }

    /// Add a sample if it is deemed more useful than the least useful sample,
    /// returning whether the retained calibration buffer changed.
    fn evaluate_sample_vec_changed(&mut self, x: Vector3<f32>, timestamp_us: u64) -> bool {
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
        let mut changed = retained != self.matrix_filled;
        if retained != self.matrix_filled {
            self.matrix_filled = retained;
            self.calibration_initialized = false;
            self.nonlinear_hard_iron_offset = Vector3::zeros();
            self.nonlinear_soft_iron_correction_factor = Matrix3::identity();
            self.nonlinear_calibration_initialized = false;
            self.using_nonlinear_fit = false;
            self.using_objective_ensemble = false;
            self.stabilizing_objective_ensemble = false;
            self.pending_sample_changes = 0;
            self.mean_distance = 0.0;
        }

        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
            return changed;
        }
        // Check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            self.add_sample_at(self.matrix_filled, x, timestamp_us);
            self.matrix_filled += 1;
            changed = true;
        }
        // Otherwise check which sample may be best to replace
        else {
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let sample_mean_dist = self.mean_distance_from_single(x.transpose());
            if low_mean_dist < sample_mean_dist {
                self.add_sample_at(low_index, x, timestamp_us);
                changed = true;
            }
        }
        changed
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
        let samples_changed = self.evaluate_sample_vec_changed(raw_mag, timestamp_us);
        if samples_changed && self.calibration_initialized {
            self.pending_sample_changes = self.pending_sample_changes.saturating_add(1);
        }
        if !self.calibration_initialized
            || self.pending_sample_changes >= CALIBRATION_SAMPLE_INTERVAL
        {
            self.perform_calibration()?;
            self.pending_sample_changes = 0;
        }
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
        let covariance = (0..sample_count).fold(Matrix3::zeros(), |sum, row| {
            let centered = self.sample(row) - sample_mean;
            sum + centered * centered.transpose()
        }) / sample_count as f32;
        let covariance_eigen = covariance.symmetric_eigen();
        let covariance_eigenvalues = covariance_eigen.eigenvalues;
        let minimum_covariance = covariance_eigenvalues.min();
        let maximum_covariance = covariance_eigenvalues.max();
        let middle_covariance =
            covariance_eigenvalues.sum() - minimum_covariance - maximum_covariance;
        let covariance_condition = maximum_covariance / minimum_covariance;
        let covariance_is_valid = minimum_covariance > f32::EPSILON
            && middle_covariance > f32::EPSILON
            && covariance_condition.is_finite()
            && covariance_condition <= MAX_SAMPLE_COVARIANCE_CONDITION;
        let trust_nonlinear_fit = covariance_is_valid
            && minimum_covariance >= middle_covariance * MIN_BCD_MINOR_TO_MIDDLE_COVARIANCE_RATIO
            && maximum_covariance >= middle_covariance * MIN_BCD_MAJOR_TO_MIDDLE_COVARIANCE_RATIO;
        let algebraic_fit = covariance_is_valid
            .then(|| self.fit_algebraic(sample_count, sample_mean))
            .flatten();
        self.update_nonlinear_candidate(sample_count, sample_mean, algebraic_fit.as_ref());
        let (
            select_nonlinear_fit,
            selected_objective_fit,
            stabilize_objective_fit,
            smooth_objective_fit,
        ) = algebraic_fit
            .as_ref()
            .map_or((false, false, false, false), |algebraic_fit| {
                let algebraic_loss =
                    self.calibration_loss(sample_count, &algebraic_fit.0, &algebraic_fit.1);
                let nonlinear_loss = self.calibration_loss(
                    sample_count,
                    &self.nonlinear_hard_iron_offset,
                    &self.nonlinear_soft_iron_correction_factor,
                );
                let factor_is_valid = self
                    .nonlinear_soft_iron_correction_factor
                    .iter()
                    .all(|value| value.is_finite())
                    && (0..3).all(|index| {
                        self.nonlinear_soft_iron_correction_factor[(index, index)] > 0.0
                    });
                let candidate_is_valid = factor_is_valid
                    && self
                        .nonlinear_hard_iron_offset
                        .iter()
                        .all(|value| value.is_finite())
                    && algebraic_loss.is_finite()
                    && nonlinear_loss.is_finite()
                    && self.center_is_bracketed(
                        sample_count,
                        &self.nonlinear_hard_iron_offset,
                        &covariance_eigen.eigenvectors,
                    );
                // Prefer an objective-improving candidate. Also accept a bounded,
                // principal-bracketed center correction at a modest residual cost
                // to reduce OLS extrapolation under incomplete coverage.
                let improves_loss = nonlinear_loss <= algebraic_loss;
                let improves_center_distance = (self.nonlinear_hard_iron_offset - sample_mean)
                    .norm_squared()
                    <= (algebraic_fit.0 - sample_mean).norm_squared();
                let retains_objective_fit = (self.using_objective_ensemble
                    || self.stabilizing_objective_ensemble && improves_center_distance)
                    && nonlinear_loss <= OBJECTIVE_ENSEMBLE_EXIT_LOSS_RATIO * algebraic_loss;
                let corrects_extrapolation = candidate_is_valid
                    && nonlinear_loss <= MAX_NONLINEAR_TO_ALGEBRAIC_LOSS_RATIO * algebraic_loss
                    && (self.nonlinear_hard_iron_offset - sample_mean).norm_squared()
                        <= MAX_NONLINEAR_CENTER_DISTANCE_RATIO.powi(2)
                            * (algebraic_fit.0 - sample_mean).norm_squared();
                    let selects_objective_fit =
                        candidate_is_valid && (improves_loss || retains_objective_fit);
                    if corrects_extrapolation && !selects_objective_fit {
                        eprintln!(
                            "direct_ratio loss={} center={} covariance_minor_middle={} covariance_major_middle={} nonlinear_offset={:?} algebraic_offset={:?} sample_mean={:?}",
                            nonlinear_loss / algebraic_loss,
                            (self.nonlinear_hard_iron_offset - sample_mean).norm()
                                / (algebraic_fit.0 - sample_mean).norm(),
                            minimum_covariance / middle_covariance,
                            maximum_covariance / middle_covariance,
                            self.nonlinear_hard_iron_offset,
                            algebraic_fit.0,
                            sample_mean,
                        );
                    }
                let stabilize_objective_fit = selects_objective_fit
                    && (self.stabilizing_objective_ensemble
                        || !trust_nonlinear_fit
                            && maximum_covariance
                                >= middle_covariance * MIN_BCD_MAJOR_TO_MIDDLE_COVARIANCE_RATIO);
                (
                    selects_objective_fit || corrects_extrapolation,
                    selects_objective_fit,
                    stabilize_objective_fit,
                    stabilize_objective_fit && !corrects_extrapolation,
                )
            });
        if select_nonlinear_fit {
            if let Some((algebraic_offset, algebraic_factor)) = algebraic_fit.as_ref() {
                let algebraic_correction = algebraic_factor.transpose() * algebraic_factor;
                let nonlinear_correction = self.nonlinear_soft_iron_correction_factor.transpose()
                    * self.nonlinear_soft_iron_correction_factor;
                let mut ensemble_correction = algebraic_correction
                    * (1.0 - NONLINEAR_ENSEMBLE_WEIGHT)
                    + nonlinear_correction * NONLINEAR_ENSEMBLE_WEIGHT;
                let mut ensemble_offset = algebraic_offset * (1.0 - NONLINEAR_ENSEMBLE_WEIGHT)
                    + self.nonlinear_hard_iron_offset * NONLINEAR_ENSEMBLE_WEIGHT;
                if self.calibration_initialized
                    && self.using_objective_ensemble
                    && self.stabilizing_objective_ensemble
                    && smooth_objective_fit
                {
                    let current_correction = self.soft_iron_correction_factor.transpose()
                        * self.soft_iron_correction_factor;
                    ensemble_correction = current_correction
                        * (1.0 - OBJECTIVE_ENSEMBLE_UPDATE_GAIN)
                        + ensemble_correction * OBJECTIVE_ENSEMBLE_UPDATE_GAIN;
                    ensemble_offset = self.hard_iron_offset
                        * (1.0 - OBJECTIVE_ENSEMBLE_UPDATE_GAIN)
                        + ensemble_offset * OBJECTIVE_ENSEMBLE_UPDATE_GAIN;
                }
                if let Some(ensemble_factor) = Self::reverse_cholesky(ensemble_correction) {
                    let factor_is_valid = ensemble_factor.iter().all(|value| value.is_finite())
                        && (0..3).all(|index| ensemble_factor[(index, index)] > 0.0);
                    if factor_is_valid && ensemble_offset.iter().all(|value| value.is_finite()) {
                        self.hard_iron_offset = ensemble_offset;
                        self.soft_iron_correction_factor = ensemble_factor;
                        self.calibration_initialized = true;
                        self.using_nonlinear_fit = true;
                        self.using_objective_ensemble = selected_objective_fit;
                        self.stabilizing_objective_ensemble |= stabilize_objective_fit;
                        return Ok(());
                    }
                }
            }
        }
        if covariance_is_valid {
            if let Some((mut offset, mut correction_factor)) = algebraic_fit {
                if self.calibration_initialized && !self.using_nonlinear_fit {
                    offset = self.hard_iron_offset * (1.0 - ALGEBRAIC_UPDATE_GAIN)
                        + offset * ALGEBRAIC_UPDATE_GAIN;
                    correction_factor = self.soft_iron_correction_factor
                        * (1.0 - ALGEBRAIC_UPDATE_GAIN)
                        + correction_factor * ALGEBRAIC_UPDATE_GAIN;
                }
                self.hard_iron_offset = offset;
                self.soft_iron_correction_factor = correction_factor;
                self.calibration_initialized = true;
                self.using_nonlinear_fit = false;
                self.using_objective_ensemble = false;
                return Ok(());
            }
        }

        self.hard_iron_offset = self.nonlinear_hard_iron_offset;
        self.soft_iron_correction_factor = self.nonlinear_soft_iron_correction_factor;
        self.calibration_initialized = self.nonlinear_calibration_initialized;
        self.using_nonlinear_fit = true;
        self.using_objective_ensemble = false;
        Ok(())
    }

    fn update_nonlinear_candidate(
        &mut self,
        sample_count: usize,
        sample_mean: Vector3<f32>,
        algebraic_fit: Option<&(Vector3<f32>, Matrix3<f32>)>,
    ) {
        let first_calibration = !self.nonlinear_calibration_initialized
            && sample_mean.iter().all(|value| value.is_finite());
        let mut offset = self.nonlinear_hard_iron_offset;
        let mut correction_factor = self.nonlinear_soft_iron_correction_factor;
        if first_calibration {
            if let Some((algebraic_offset, algebraic_factor)) = algebraic_fit {
                offset = *algebraic_offset;
                correction_factor = *algebraic_factor;
            } else {
                offset = sample_mean;
                let mean_squared_radius = (0..sample_count)
                    .map(|row| (self.sample(row) - sample_mean).norm_squared())
                    .sum::<f32>()
                    / sample_count as f32;
                let rho = mean_squared_radius.sqrt().sqrt();
                if rho.is_finite() && rho > f32::EPSILON {
                    correction_factor = Matrix3::identity() * rho.recip();
                }
            }
        }
        let warm_start_relaxation =
            (sample_count as f32 / CALIBRATION_SAMPLE_INTERVAL as f32).max(1.0);
        let offset_step_relaxation = if first_calibration {
            3.0
        } else {
            warm_start_relaxation
        };
        let factor_step_relaxation = if first_calibration {
            6.0
        } else {
            warm_start_relaxation
        };

        (offset, correction_factor) = self.refine_nonlinear_candidate(
            sample_count,
            offset,
            correction_factor,
            offset_step_relaxation,
            factor_step_relaxation,
        );

        self.nonlinear_hard_iron_offset = offset;
        self.nonlinear_soft_iron_correction_factor = correction_factor;
        self.nonlinear_calibration_initialized = true;
    }

    fn refine_nonlinear_candidate(
        &self,
        sample_count: usize,
        mut offset: Vector3<f32>,
        mut correction_factor: Matrix3<f32>,
        offset_step_relaxation: f32,
        factor_step_relaxation: f32,
    ) -> (Vector3<f32>, Matrix3<f32>) {
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

        let mean_corrected_norm_squared = (0..sample_count)
            .map(|row| {
                Self::apply_correction(self.sample(row) - offset, &correction_factor).norm_squared()
            })
            .sum::<f32>()
            / sample_count as f32;
        let factor_scale = mean_corrected_norm_squared.sqrt().sqrt().recip();
        if factor_scale.is_finite() {
            correction_factor *= factor_scale;
        }
        (offset, correction_factor)
    }

    fn sample(&self, row: usize) -> Vector3<f32> {
        // TODO: this should be a linear algebra operation, avoid elementwise operations
        Vector3::new(
            self.matrix[(row, 0)],
            self.matrix[(row, 1)],
            self.matrix[(row, 2)],
        )
    }

    fn calibration_loss(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        correction_factor: &Matrix3<f32>,
    ) -> f32 {
        ((0..sample_count)
            .map(|row| {
                let residual = Self::apply_correction(self.sample(row) - offset, correction_factor)
                    .norm()
                    - 1.0;
                residual * residual
            })
            .sum::<f32>()
            / sample_count as f32)
            .sqrt()
    }

    fn center_is_bracketed(
        &self,
        sample_count: usize,
        offset: &Vector3<f32>,
        principal_axes: &Matrix3<f32>,
    ) -> bool {
        let mut has_negative = [false; 3];
        let mut has_positive = [false; 3];
        for row in 0..sample_count {
            let projection = principal_axes.transpose() * (self.sample(row) - offset);
            for axis in 0..3 {
                has_negative[axis] |= projection[axis] < 0.0;
                has_positive[axis] |= projection[axis] > 0.0;
            }
        }
        (0..3).all(|axis| has_negative[axis] && has_positive[axis])
    }

    fn fit_algebraic(
        &self,
        sample_count: usize,
        sample_mean: Vector3<f32>,
    ) -> Option<(Vector3<f32>, Matrix3<f32>)> {
        let sample_scale = ((0..sample_count)
            .map(|row| (self.sample(row) - sample_mean).norm_squared())
            .sum::<f32>()
            / sample_count as f32)
            .sqrt();
        if !sample_scale.is_finite() || sample_scale <= f32::EPSILON {
            return None;
        }
        let mut normal = SMatrix::<f32, 9, 9>::zeros();
        let mut rhs = SVector::<f32, 9>::zeros();
        for row in 0..sample_count {
            let sample = (self.sample(row) - sample_mean) / sample_scale;
            let design = Self::algebraic_design(sample);
            normal += design * design.transpose();
            rhs += design;
        }
        let normal_eigenvalues = normal.symmetric_eigenvalues();
        let normal_condition = normal_eigenvalues.max() / normal_eigenvalues.min();
        if normal_eigenvalues.min() <= f32::EPSILON
            || !normal_condition.is_finite()
            || normal_condition > MAX_ALGEBRAIC_NORMAL_CONDITION
        {
            return None;
        }
        let parameters = Self::solve_algebraic_parameters(normal, rhs, ALGEBRAIC_FIT_DAMPING)?;
        let shape = Self::algebraic_shape(&parameters);
        let linear = Vector3::new(parameters[6], parameters[7], parameters[8]);
        let normalized_offset = -0.5 * shape.try_inverse()? * linear;
        let radius_squared = 1.0 + normalized_offset.dot(&(shape * normalized_offset));
        if !radius_squared.is_finite() || radius_squared <= f32::EPSILON {
            return None;
        }
        let normalized_shape = shape / radius_squared;
        let eigen = normalized_shape.symmetric_eigen();
        if !eigen
            .eigenvalues
            .iter()
            .all(|value| value.is_finite() && *value > f32::EPSILON)
        {
            return None;
        }
        let correction = eigen.eigenvectors
            * Matrix3::from_diagonal(&eigen.eigenvalues.map(f32::sqrt))
            * eigen.eigenvectors.transpose()
            / sample_scale;
        let offset = sample_mean + normalized_offset * sample_scale;
        let correction_factor = Self::reverse_cholesky(correction)?;
        offset
            .iter()
            .chain(correction_factor.iter())
            .all(|value| value.is_finite())
            .then_some((offset, correction_factor))
    }

    fn algebraic_design(sample: Vector3<f32>) -> SVector<f32, 9> {
        SVector::<f32, 9>::from_row_slice(&[
            sample.x * sample.x,
            sample.y * sample.y,
            sample.z * sample.z,
            2.0 * sample.x * sample.y,
            2.0 * sample.x * sample.z,
            2.0 * sample.y * sample.z,
            sample.x,
            sample.y,
            sample.z,
        ])
    }

    fn algebraic_shape(parameters: &SVector<f32, 9>) -> Matrix3<f32> {
        Matrix3::new(
            parameters[0],
            parameters[3],
            parameters[4],
            parameters[3],
            parameters[1],
            parameters[5],
            parameters[4],
            parameters[5],
            parameters[2],
        )
    }

    fn solve_algebraic_parameters(
        mut normal: SMatrix<f32, 9, 9>,
        mut rhs: SVector<f32, 9>,
        damping_factor: f32,
    ) -> Option<SVector<f32, 9>> {
        let damping = damping_factor * normal.trace().max(1.0) / 9.0;
        normal += SMatrix::<f32, 9, 9>::identity() * damping;
        rhs[0] += damping;
        rhs[1] += damping;
        rhs[2] += damping;
        let parameters = normal.cholesky()?.solve(&rhs);
        parameters
            .iter()
            .all(|value| value.is_finite())
            .then_some(parameters)
    }

    fn reverse_cholesky(matrix: Matrix3<f32>) -> Option<Matrix3<f32>> {
        let l22 = matrix[(2, 2)].sqrt();
        let l21 = matrix[(1, 2)] / l22;
        let l20 = matrix[(0, 2)] / l22;
        let l11 = (matrix[(1, 1)] - l21 * l21).sqrt();
        let l10 = (matrix[(0, 1)] - l20 * l21) / l11;
        let l00 = (matrix[(0, 0)] - l10 * l10 - l20 * l20).sqrt();
        let factor = Matrix3::new(l00, 0.0, 0.0, l10, l11, 0.0, l20, l21, l22);
        factor
            .iter()
            .all(|value| value.is_finite())
            .then_some(factor)
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

        let damping = FACTOR_DAMPING * sample_count as f32;
        for (index, &(matrix_row, matrix_column)) in parameter_indices.iter().enumerate() {
            let value = correction_factor[(matrix_row, matrix_column)];
            let delta = (gradient[index] / (curvature[index] + damping) / step_relaxation)
                .clamp(-MAX_PARAMETER_STEP, MAX_PARAMETER_STEP);
            let candidate = value - delta;
            let preserves_positive_diagonal = matrix_row != matrix_column || candidate > 0.0;
            if delta.is_finite() && candidate.is_finite() && preserves_positive_diagonal {
                correction_factor[(matrix_row, matrix_column)] = candidate;
            }
        }
    }
}
