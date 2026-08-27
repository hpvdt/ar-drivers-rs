use nalgebra::{Matrix3, SMatrix, SVector, SymmetricEigen, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
/// Design sum of the retained direction features backing the coverage score.
pub(super) type DesignMatrix =
    SMatrix<f32, CALIBRATION_PARAMETER_COUNT, CALIBRATION_PARAMETER_COUNT>;
const SHAPE_REGULARIZATION: f32 = 1.0e-3;
/// Scale of the regularization target shape, in units of the identity.
/// Algebraic ellipsoid fits under noise systematically inflate the ellipsoid
/// (underestimate the eigenvalues of the shape matrix), so the prior centers
/// on a shape larger than the ideal sphere to counter that bias.
const SHAPE_PRIOR_SCALE: f32 = 2.0;
const MAX_CORRECTION_CONDITION: f32 = 1.0e1;
const MAX_RADIAL_RMS: f32 = 0.1;
/// Confidence required for a working candidate to advance the publication
/// streak. With E-optimality coverage this sits in the gap between a
/// calibrator whose retained motion genuinely fills a broad region (whose
/// live confidence plateaus at 0.042 or above even when it never visits a
/// third axis) and one still confined to the first motion segments (whose
/// near-planar support leaves the design matrix rank-deficient and the
/// confidence below 0.02 sustained).
pub(super) const MIN_PUBLICATION_CONFIDENCE: f32 = 0.03;
/// Confidence floor below which the publication streak resets. This
/// hysteresis keeps a qualifying candidate from losing its streak to
/// threshold jitter: a short dip in live quality (for example while the
/// optimizer absorbs a newly visited motion segment) pauses the streak
/// instead of restarting it, while a genuine quality collapse resets it.
const PUBLICATION_STREAK_RESET_CONFIDENCE: f32 = 0.02;
pub(super) const MIN_PUBLICATION_STREAK: usize = 110;
/// Uniform-sphere reference for directional coverage: the smallest
/// eigenvalue of `E[phi(d) phi(d)^T]` over uniformly distributed unit
/// directions, where `phi` is the quadratic feature vector shared with the
/// ellipsoid fit. A fully isotropic cache scores 1 against this reference.
const COVERAGE_LAMBDA_REF: f32 = 2.0 / 15.0;
const MIN_MAG_NORM: f32 = 0.4;
const DEFAULT_GRAVITY_WEIGHT: f32 = 0.01;
const DEFAULT_MINIBATCH_SIZE: usize = 32;
/// Cache-only replay updates run per valid sample while the calibration is
/// still unpublished. They let the cold-start optimizer take several gradient
/// steps per arriving sample without waiting for new data.
const DEFAULT_REPLAY_UPDATES: usize = 4;
/// Replay minibatches are smaller than the sample-anchored minibatch because
/// several of them run per sample and each one re-evaluates its objective
/// during the bounded half-step search.
const DEFAULT_REPLAY_MINIBATCH_SIZE: usize = 8;
const ONLINE_INITIAL_LEARNING_RATE: f32 = 0.5;
/// Learning-rate annealing timescale, in optimizer steps. The target ellipsoid
/// is not stationary: it keeps moving as long as cache replacements improve
/// the sample coverage, which under near-planar motion continues well past the
/// `N`-sample cache fill. The rate must therefore stay high enough through and
/// beyond the fill for the optimizer to track the moving convex optimum. A
/// timescale far below the fill time collapses the rate before convergence and
/// strands the working shape far from the optimum (seen as >18 deg worst-case
/// SimMotion-integration error with a near-planar seed at 64 steps).
const ONLINE_LEARNING_RATE_DECAY_STEPS: f32 = 128.0;
const ONLINE_MIN_LEARNING_RATE: f32 = 0.01;
const ONLINE_MAX_STEP_NORM: f32 = 0.5;
const ONLINE_SCALE_EPSILON: f32 = 1.0e-4;
const ONLINE_BACKTRACK_STEPS: usize = 12;
const ONLINE_PRNG_SEED: u64 = 0x9E37_79B9_7F4A_7C15;
/// Number of neighbor entries cached per buffered sample row: the `k` nearest
/// squared distances plus an overshoot pad that absorbs neighbor churn before
/// an O(N) row rescan becomes necessary. Configurations with `num_neighbors`
/// above this capacity bypass the cache and scan rows directly.
const NEIGHBOR_CACHE_CAPACITY: usize = 8;

/// Squared distance from one buffered sample row to another, addressed by row
/// index.
#[derive(Clone, Copy)]
struct NeighborEntry {
    squared_distance: f32,
    row: u32,
}

impl NeighborEntry {
    /// Placeholder for cache slots past the row's trusted prefix length; such
    /// slots are never read.
    const EMPTY: Self = Self {
        squared_distance: f32::INFINITY,
        row: 0,
    };
}

#[derive(Clone, Copy)]
struct MinibatchSpec {
    /// The arriving observation anchoring a sample-triggered update.
    /// Cache-replay updates leave this empty and draw every observation from
    /// the retained rows.
    current_sample: Option<Vector3<f32>>,
    current_gravity: Option<Vector3<f32>>,
    accepted_row: Option<usize>,
    random_draws: usize,
    random_state: u64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CalibrationCandidate {
    offset: Vector3<f32>,
    correction: Matrix3<f32>,
}

/// Result of evaluating one FRD magnetometer observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagCalibrationResult {
    // TODO: this result should also contain every factor used to compute the confidence score
    /// Current bounded calibration quality in `[0, 1]`.
    pub confidence: f32,
    /// Corrected and normalized FRD magnetic direction, produced by the
    /// published correction; `None` while no correction has passed the live
    /// quality gates yet.
    pub direction: Option<Vector3<f32>>,
}

/// Online regularized ellipsoid fit for a hard-iron offset and full SPD
/// soft-iron correction from a fixed, diverse sample buffer.
pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 3>,
    gravity_directions: [Option<Vector3<f32>>; N],
    sample_timestamps_us: [u64; N],
    matrix_filled: usize,
    hard_iron_offset: Vector3<f32>,
    soft_iron_correction: Matrix3<f32>,
    calibration_initialized: bool,
    mean_distance: f32,
    /// Per-row incremental k-nearest-neighbor cache for the diversity
    /// heuristic. Invariant: `neighbor_cache[i][..neighbor_cache_len[i]]`
    /// lists, in ascending squared distance, the true nearest other buffered
    /// rows of row `i` (the row itself is excluded by index), and every
    /// buffered row not listed is at least as far as the last listed entry.
    /// The trusted prefix is updated in place on append and replace, remapped
    /// on expiry compaction, and rebuilt with an O(N) scan once it shrinks
    /// below `k`.
    neighbor_cache: [[NeighborEntry; NEIGHBOR_CACHE_CAPACITY]; N],
    neighbor_cache_len: [u8; N],
    k: usize,
    max_sample_lifespan_us: u64,
    gravity_weight: f32,
    parameters: SVector<f32, CALIBRATION_PARAMETER_COUNT>,
    normalization_mean: Vector3<f32>,
    normalization_radius: f32,
    normalization_initialized: bool,
    gravity_projection: f32,
    gravity_projection_initialized: bool,
    minibatch_size: usize,
    replay_updates: usize,
    replay_minibatch_size: usize,
    prng_state: u64,
    optimizer_steps: u64,
    raw_sample_sum: Vector3<f64>,
    raw_outer_product_sum: Matrix3<f64>,
    radial_residual_mean_square: Option<f32>,
    confidence: f32,
    publication_quality_streak: usize,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::zeros(),
            gravity_directions: std::array::from_fn(|_| None),
            sample_timestamps_us: [0; N],
            matrix_filled: Default::default(),
            hard_iron_offset: Vector3::zeros(),
            soft_iron_correction: Matrix3::identity(),
            calibration_initialized: false,
            mean_distance: Default::default(),
            neighbor_cache: [[NeighborEntry::EMPTY; NEIGHBOR_CACHE_CAPACITY]; N],
            neighbor_cache_len: [0; N],
            k: 2, // Works well in testing
            max_sample_lifespan_us: 60 * 60 * 1_000_000,
            gravity_weight: DEFAULT_GRAVITY_WEIGHT,
            parameters: Self::parameter_prior(),
            normalization_mean: Vector3::zeros(),
            normalization_radius: 0.0,
            normalization_initialized: false,
            gravity_projection: 0.0,
            gravity_projection_initialized: false,
            minibatch_size: DEFAULT_MINIBATCH_SIZE.min(N.max(1)),
            replay_updates: DEFAULT_REPLAY_UPDATES,
            replay_minibatch_size: DEFAULT_REPLAY_MINIBATCH_SIZE.min(N.max(1)),
            prng_state: ONLINE_PRNG_SEED,
            optimizer_steps: 0,
            raw_sample_sum: Vector3::zeros(),
            raw_outer_product_sum: Matrix3::zeros(),
            radial_residual_mean_square: None,
            confidence: 0.0,
            publication_quality_streak: 0,
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

    /// Configure the relative weight of the gravity-consistency residual.
    /// The default is 0.01; zero disables the ellipsoid-normal gravity
    /// surrogate.
    pub fn gravity_weight(self, gravity_weight: f32) -> Self {
        Self {
            gravity_weight: if gravity_weight.is_finite() {
                gravity_weight.max(0.0)
            } else {
                0.0
            },
            ..self
        }
    }

    /// Configure the maximum number of observations used by each online
    /// optimizer update. The current valid observation is always included. The
    /// default is 32, capped by the sample-buffer capacity.
    pub fn minibatch_size(self, minibatch_size: usize) -> Self {
        Self {
            minibatch_size: minibatch_size.clamp(1, N.max(1)),
            ..self
        }
    }

    /// Configure the number of additional cache-only optimizer updates run
    /// with each valid sample while the calibration is still unpublished.
    /// Replay draws its whole minibatch from the retained rows, so it never
    /// requires the arriving sample; it accelerates cold-start convergence at
    /// a small pre-publication computation cost. The default is 4; zero
    /// disables replay.
    pub fn replay_updates(self, replay_updates: usize) -> Self {
        Self {
            replay_updates,
            ..self
        }
    }

    /// Configure the number of retained-row observations used by each
    /// cache-replay update. The default is 8, smaller than the
    /// sample-anchored minibatch, capped by the sample-buffer capacity.
    pub fn replay_minibatch_size(self, replay_minibatch_size: usize) -> Self {
        Self {
            replay_minibatch_size: replay_minibatch_size.clamp(1, N.max(1)),
            ..self
        }
    }

    fn parameter_prior() -> SVector<f32, CALIBRATION_PARAMETER_COUNT> {
        SVector::from_row_slice(&[
            SHAPE_PRIOR_SCALE,
            SHAPE_PRIOR_SCALE,
            SHAPE_PRIOR_SCALE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ])
    }

    fn shape_and_linear(
        parameters: &SVector<f32, CALIBRATION_PARAMETER_COUNT>,
    ) -> (Matrix3<f32>, Vector3<f32>) {
        (
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
            ),
            Vector3::new(parameters[6], parameters[7], parameters[8]),
        )
    }

    fn features(sample: Vector3<f32>) -> SVector<f32, CALIBRATION_PARAMETER_COUNT> {
        SVector::from_row_slice(&[
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

    /// Features of the projection of gravity onto the ellipsoid normal
    /// `Q * sample + q / 2`. The projection is linear in the nine ellipsoid
    /// parameters, keeping the combined online objective convex and quadratic.
    fn gravity_features(
        sample: Vector3<f32>,
        gravity: Vector3<f32>,
    ) -> SVector<f32, CALIBRATION_PARAMETER_COUNT> {
        SVector::from_row_slice(&[
            gravity.x * sample.x,
            gravity.y * sample.y,
            gravity.z * sample.z,
            gravity.x * sample.y + gravity.y * sample.x,
            gravity.x * sample.z + gravity.z * sample.x,
            gravity.y * sample.z + gravity.z * sample.y,
            0.5 * gravity.x,
            0.5 * gravity.y,
            0.5 * gravity.z,
        ])
    }

    fn regularization_loss(parameters: &SVector<f32, CALIBRATION_PARAMETER_COUNT>) -> f32 {
        let prior = Self::parameter_prior();
        let weights = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0];
        0.5 * SHAPE_REGULARIZATION
            * (0..6)
                .map(|index| weights[index] * (parameters[index] - prior[index]).powi(2))
                .sum::<f32>()
    }

    fn reset_working_state(&mut self) {
        self.parameters = Self::parameter_prior();
        self.gravity_projection = 0.0;
        self.gravity_projection_initialized = false;
        self.optimizer_steps = 0;
        self.radial_residual_mean_square = None;
        self.confidence = 0.0;
        self.publication_quality_streak = 0;
    }

    fn add_raw_moment(&mut self, sample: Vector3<f32>) {
        let sample = sample.cast::<f64>();
        self.raw_sample_sum += sample;
        self.raw_outer_product_sum += sample * sample.transpose();
    }

    fn remove_raw_moment(&mut self, sample: Vector3<f32>) {
        let sample = sample.cast::<f64>();
        self.raw_sample_sum -= sample;
        self.raw_outer_product_sum -= sample * sample.transpose();
    }

    fn clear_raw_moments(&mut self) {
        self.raw_sample_sum = Vector3::zeros();
        self.raw_outer_product_sum = Matrix3::zeros();
    }

    fn raw_mean_and_covariance(&self) -> Option<(Vector3<f32>, Matrix3<f32>)> {
        if self.matrix_filled == 0 {
            return None;
        }
        let count = self.matrix_filled as f64;
        let mean = self.raw_sample_sum / count;
        let covariance = self.raw_outer_product_sum / count - mean * mean.transpose();
        let covariance = 0.5 * (covariance + covariance.transpose());
        let mean = mean.cast::<f32>();
        let covariance = covariance.cast::<f32>();
        if mean.iter().all(|value| value.is_finite())
            && covariance.iter().all(|value| value.is_finite())
        {
            Some((mean, covariance))
        } else {
            None
        }
    }

    /// Recomputes the current cache normalization and analytically transforms
    /// the working quadratic equation into it. A failed transform resets only
    /// unpublished optimizer state.
    fn refresh_normalization(&mut self) {
        let Some((sample_mean, covariance)) = self.raw_mean_and_covariance() else {
            self.normalization_mean = Vector3::zeros();
            self.normalization_radius = 0.0;
            self.normalization_initialized = false;
            self.reset_working_state();
            return;
        };
        let radius_squared = covariance.trace();
        let radius = radius_squared.sqrt();
        if !sample_mean.iter().all(|value| value.is_finite())
            || !radius.is_finite()
            || radius <= f32::EPSILON
        {
            self.normalization_mean = sample_mean;
            self.normalization_radius = radius;
            self.normalization_initialized = false;
            self.reset_working_state();
            return;
        }

        if self.normalization_initialized {
            let (shape, linear) = Self::shape_and_linear(&self.parameters);
            let shift = (sample_mean - self.normalization_mean) / self.normalization_radius;
            let scale = radius / self.normalization_radius;
            let equation_scale = 1.0 - shift.dot(&(shape * shift)) - linear.dot(&shift);
            let rebased_shape = scale * scale / equation_scale * shape;
            let rebased_linear = scale / equation_scale * (linear + 2.0 * shape * shift);
            let mut rebased = self.parameters;
            rebased[0] = rebased_shape[(0, 0)];
            rebased[1] = rebased_shape[(1, 1)];
            rebased[2] = rebased_shape[(2, 2)];
            rebased[3] = rebased_shape[(0, 1)];
            rebased[4] = rebased_shape[(0, 2)];
            rebased[5] = rebased_shape[(1, 2)];
            rebased[6] = rebased_linear.x;
            rebased[7] = rebased_linear.y;
            rebased[8] = rebased_linear.z;
            let gravity_projection = scale / equation_scale * self.gravity_projection;
            if equation_scale.is_finite()
                && equation_scale > f32::EPSILON
                && rebased.iter().all(|value| value.is_finite())
                && (!self.gravity_projection_initialized || gravity_projection.is_finite())
            {
                self.parameters = rebased;
                self.gravity_projection = gravity_projection;
            } else {
                self.reset_working_state();
            }
        } else {
            self.reset_working_state();
        }

        self.normalization_mean = sample_mean;
        self.normalization_radius = radius;
        self.normalization_initialized = true;
    }

    fn normalized_sample(&self, sample: Vector3<f32>) -> Vector3<f32> {
        (sample - self.normalization_mean) / self.normalization_radius
    }

    fn next_random(state: &mut u64) -> u64 {
        *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut value = *state;
        value = (value ^ (value >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        value ^ (value >> 31)
    }

    fn random_cache_row(
        state: &mut u64,
        sample_count: usize,
        excluded_row: Option<usize>,
    ) -> Option<usize> {
        let eligible_count = sample_count.saturating_sub(usize::from(excluded_row.is_some()));
        if eligible_count == 0 {
            return None;
        }
        let mut row = (Self::next_random(state) % eligible_count as u64) as usize;
        if excluded_row.is_some_and(|excluded| row >= excluded) {
            row += 1;
        }
        Some(row)
    }

    fn initialize_gravity_projection(
        &mut self,
        current_sample: Vector3<f32>,
        current_gravity: Option<Vector3<f32>>,
    ) {
        if self.gravity_projection_initialized || self.gravity_weight == 0.0 {
            return;
        }
        let observation = current_gravity
            .map(|gravity| (current_sample, gravity))
            .or_else(|| {
                (0..self.matrix_filled).find_map(|row| {
                    self.gravity_directions[row].map(|gravity| (self.sample(row), gravity))
                })
            });
        if let Some((sample, gravity)) = observation {
            let features = Self::gravity_features(self.normalized_sample(sample), gravity);
            let projection = features.dot(&self.parameters);
            if projection.is_finite() {
                self.gravity_projection = projection;
                self.gravity_projection_initialized = true;
            }
        }
    }

    fn minibatch_objective(
        &self,
        parameters: &SVector<f32, CALIBRATION_PARAMETER_COUNT>,
        gravity_projection: f32,
        minibatch: MinibatchSpec,
    ) -> f32 {
        let mut random_state = minibatch.random_state;
        let mut radial_squared = 0.0;
        let mut gravity_squared = 0.0;
        let mut observation_count = 0;
        let mut gravity_count = 0;
        let mut add_observation = |sample: Vector3<f32>, gravity: Option<Vector3<f32>>| {
            let normalized = self.normalized_sample(sample);
            let residual = Self::features(normalized).dot(parameters) - 1.0;
            radial_squared += residual * residual;
            observation_count += 1;
            if let Some(gravity) = gravity.filter(|_| self.gravity_projection_initialized) {
                let residual = Self::gravity_features(normalized, gravity).dot(parameters)
                    - gravity_projection;
                gravity_squared += residual * residual;
                gravity_count += 1;
            }
        };

        if let Some(sample) = minibatch.current_sample {
            add_observation(sample, minibatch.current_gravity);
        }
        for _ in 0..minibatch.random_draws {
            let Some(row) = Self::random_cache_row(
                &mut random_state,
                self.matrix_filled,
                minibatch.accepted_row,
            ) else {
                break;
            };
            add_observation(self.sample(row), self.gravity_directions[row]);
        }

        let mut objective =
            0.5 * radial_squared / observation_count as f32 + Self::regularization_loss(parameters);
        if gravity_count > 0 {
            objective += 0.5 * self.gravity_weight * gravity_squared / gravity_count as f32;
        }
        objective
    }

    /// Applies one bounded normalized-SGD update anchored to the current
    /// valid sample, followed, while the calibration is still unpublished, by
    /// the configured number of cache-replay updates drawn purely from the
    /// retained rows.
    fn update_online_optimizer(
        &mut self,
        current_sample: Vector3<f32>,
        current_gravity: Option<Vector3<f32>>,
        accepted_row: Option<usize>,
    ) {
        if !self.normalization_initialized {
            return;
        }
        self.initialize_gravity_projection(current_sample, current_gravity);

        let random_draws = if self.matrix_filled > usize::from(accepted_row.is_some()) {
            self.minibatch_size.saturating_sub(1)
        } else {
            0
        };
        if self.apply_minibatch_update(MinibatchSpec {
            current_sample: Some(current_sample),
            current_gravity,
            accepted_row,
            random_draws,
            random_state: self.prng_state,
        }) {
            self.optimizer_steps += 1;
        }

        // Cold-start cache replay: the arriving sample is never a required
        // member of a replay minibatch; once retained, it is an ordinary
        // cache row that replay may draw like any other. Replay ramps in with
        // the retained fraction: repeatedly fitting a small, low-coverage
        // cache overfits it and can strand the working shape outside the
        // publishable region, while a nearly full cache is representative
        // enough to converge against. Replay steps reuse the current
        // learning rate without advancing its schedule, so annealing stays
        // tied to the rate of arriving data rather than to compute.
        if self.calibration_initialized || self.matrix_filled == 0 {
            return;
        }
        let replay_count = self.replay_updates.saturating_mul(self.matrix_filled) / N.max(1);
        for _ in 0..replay_count {
            self.apply_minibatch_update(MinibatchSpec {
                current_sample: None,
                current_gravity: None,
                accepted_row: None,
                random_draws: self.replay_minibatch_size,
                random_state: self.prng_state,
            });
        }
    }

    /// Applies one bounded normalized-SGD update over the given minibatch,
    /// advancing the private draw state past the sampled rows. Returns whether
    /// a finite objective-lowering step was accepted; an update that finds no
    /// observation or no usable descent direction leaves the working state
    /// unchanged.
    fn apply_minibatch_update(&mut self, minibatch: MinibatchSpec) -> bool {
        let mut next_random_state = minibatch.random_state;
        let mut gradient = SVector::<f32, CALIBRATION_PARAMETER_COUNT>::zeros();
        let mut gradient_scale = SVector::<f32, CALIBRATION_PARAMETER_COUNT>::zeros();
        let mut gravity_gradient = SVector::<f32, CALIBRATION_PARAMETER_COUNT>::zeros();
        let mut gravity_scale = SVector::<f32, CALIBRATION_PARAMETER_COUNT>::zeros();
        let mut gravity_projection_gradient = 0.0;
        let mut observation_count = 0;
        let mut gravity_count = 0;
        let parameters = self.parameters;
        let gravity_projection = self.gravity_projection;
        let mut add_observation = |sample: Vector3<f32>, gravity: Option<Vector3<f32>>| {
            let normalized = self.normalized_sample(sample);
            let features = Self::features(normalized);
            let residual = features.dot(&parameters) - 1.0;
            gradient += residual * features;
            gradient_scale += features.component_mul(&features);
            observation_count += 1;
            if let Some(gravity) = gravity.filter(|_| self.gravity_projection_initialized) {
                let features = Self::gravity_features(normalized, gravity);
                let residual = features.dot(&parameters) - gravity_projection;
                gravity_gradient += residual * features;
                gravity_scale += features.component_mul(&features);
                gravity_projection_gradient -= residual;
                gravity_count += 1;
            }
        };

        if let Some(sample) = minibatch.current_sample {
            add_observation(sample, minibatch.current_gravity);
        }
        for _ in 0..minibatch.random_draws {
            let Some(row) = Self::random_cache_row(
                &mut next_random_state,
                self.matrix_filled,
                minibatch.accepted_row,
            ) else {
                break;
            };
            add_observation(self.sample(row), self.gravity_directions[row]);
        }
        self.prng_state = next_random_state;
        if observation_count == 0 {
            return false;
        }

        gradient /= observation_count as f32;
        gradient_scale /= observation_count as f32;
        if gravity_count > 0 {
            gradient += self.gravity_weight * gravity_gradient / gravity_count as f32;
            gradient_scale += self.gravity_weight * gravity_scale / gravity_count as f32;
            gravity_projection_gradient *= self.gravity_weight / gravity_count as f32;
        } else {
            gravity_projection_gradient = 0.0;
        }

        let prior = Self::parameter_prior();
        for (index, weight) in [1.0, 1.0, 1.0, 2.0, 2.0, 2.0].into_iter().enumerate() {
            gradient[index] += SHAPE_REGULARIZATION * weight * (parameters[index] - prior[index]);
            gradient_scale[index] += SHAPE_REGULARIZATION * weight;
        }
        gradient_scale.add_scalar_mut(ONLINE_SCALE_EPSILON);
        let direction = gradient.component_div(&gradient_scale);
        let gravity_direction = if gravity_count > 0 && self.gravity_weight > 0.0 {
            gravity_projection_gradient / (self.gravity_weight + ONLINE_SCALE_EPSILON)
        } else {
            0.0
        };
        let direction_norm =
            (direction.norm_squared() + gravity_direction * gravity_direction).sqrt();
        if !direction_norm.is_finite() || direction_norm <= f32::EPSILON {
            return false;
        }

        let old_objective = self.minibatch_objective(&parameters, gravity_projection, minibatch);
        let learning_rate = (ONLINE_INITIAL_LEARNING_RATE
            / (1.0 + self.optimizer_steps as f32 / ONLINE_LEARNING_RATE_DECAY_STEPS))
            .max(ONLINE_MIN_LEARNING_RATE);
        let mut step = learning_rate.min(ONLINE_MAX_STEP_NORM / direction_norm);
        for _ in 0..ONLINE_BACKTRACK_STEPS {
            let candidate = parameters - step * direction;
            let candidate_gravity_projection = gravity_projection - step * gravity_direction;
            let objective =
                self.minibatch_objective(&candidate, candidate_gravity_projection, minibatch);
            if candidate.iter().all(|value| value.is_finite())
                && candidate_gravity_projection.is_finite()
                && objective.is_finite()
                && objective < old_objective
            {
                self.parameters = candidate;
                self.gravity_projection = candidate_gravity_projection;
                return true;
            }
            step *= 0.5;
        }
        false
    }

    /// Computes squared distances from `x` to the first `count` rows of the
    /// sample buffer. Entries at and beyond `count` are set to infinity so
    /// selection never picks them.
    fn squared_distances_to(&self, x: Vector3<f32>, count: usize) -> [f32; N] {
        let mut squared_dists = [f32::INFINITY; N];
        for (j, dist) in squared_dists.iter_mut().enumerate().take(count) {
            let diff = x - self.sample(j);
            *dist = diff.dot(&diff);
        }
        squared_dists
    }

    /// Mean distance over the `k` smallest entries of `squared`, selected in
    /// O(n) with a partial sort; `squared` is reordered in the process. The
    /// square root is deferred until after selection, so only the `k`
    /// selected entries are sqrt'd. Returns infinity for `k == 0`.
    fn mean_of_smallest(squared: &mut [f32], k: usize) -> f32 {
        if k == 0 {
            return f32::INFINITY;
        }
        squared.select_nth_unstable_by(k - 1, |a, b| a.total_cmp(b));
        let smallest = &mut squared[..k];
        smallest.sort_unstable_by(|a, b| a.total_cmp(b));
        smallest.iter().rev().fold(0., |acc, &d| acc + d.sqrt()) / k as f32
    }

    /// Inserts `entry` into a row's neighbor cache, keeping it sorted and
    /// bounded by the capacity. An entry ranking beyond the trusted prefix is
    /// only appended when the cache currently covers every other buffered row
    /// (`complete`); otherwise it is dropped, because an uncached row may
    /// legitimately be closer and would silently break the prefix invariant.
    fn cache_insert(
        cache: &mut [NeighborEntry; NEIGHBOR_CACHE_CAPACITY],
        len: &mut u8,
        entry: NeighborEntry,
        complete: bool,
    ) {
        let count = *len as usize;
        let rank = cache[..count].partition_point(|e| e.squared_distance < entry.squared_distance);
        if rank < count {
            let shift_end = count.min(NEIGHBOR_CACHE_CAPACITY - 1);
            cache.copy_within(rank..shift_end, rank + 1);
            cache[rank] = entry;
            if count < NEIGHBOR_CACHE_CAPACITY {
                *len += 1;
            }
        } else if complete && count < NEIGHBOR_CACHE_CAPACITY {
            cache[count] = entry;
            *len += 1;
        }
    }

    /// Removes the entry referencing `row` from a neighbor cache, if present.
    /// Dropping an entry keeps the remaining prefix trusted.
    fn cache_remove(cache: &mut [NeighborEntry; NEIGHBOR_CACHE_CAPACITY], len: &mut u8, row: u32) {
        let count = *len as usize;
        if let Some(position) = cache[..count].iter().position(|e| e.row == row) {
            cache.copy_within(position + 1..count, position);
            *len -= 1;
        }
    }

    /// Rebuilds a row's neighbor cache from scratch: the
    /// `NEIGHBOR_CACHE_CAPACITY` smallest squared distances among rows
    /// `0..count`, skipping the row's own entry by index. O(N).
    fn reset_row_cache(&mut self, row: usize, squared_dists: &[f32; N], count: usize) {
        let mut entries = [NeighborEntry::EMPTY; N];
        let mut entry_count = 0;
        for (j, &squared_distance) in squared_dists.iter().enumerate().take(count) {
            if j == row {
                continue;
            }
            entries[entry_count] = NeighborEntry {
                squared_distance,
                row: j as u32,
            };
            entry_count += 1;
        }
        let take = NEIGHBOR_CACHE_CAPACITY.min(entry_count);
        if take > 0 {
            entries[..entry_count].select_nth_unstable_by(take - 1, |a, b| {
                a.squared_distance.total_cmp(&b.squared_distance)
            });
            entries[..take]
                .sort_unstable_by(|a, b| a.squared_distance.total_cmp(&b.squared_distance));
        }
        self.neighbor_cache[row][..take].copy_from_slice(&entries[..take]);
        self.neighbor_cache_len[row] = take as u8;
    }

    /// Recomputes a row's neighbor cache when its trusted prefix has shrunk
    /// below `k`. Only called with a full buffer.
    fn rebuild_row_cache(&mut self, row: usize) {
        let squared_dists = self.squared_distances_to(self.sample(row), N);
        self.reset_row_cache(row, &squared_dists, N);
    }

    /// Mean distance of a buffered row to its `k` nearest other rows, served
    /// from the incremental neighbor cache. A smaller number means the point
    /// is "similar" to its neighbors. Rows whose trusted prefix has shrunk
    /// below `k` are rescanned in O(N) first; configurations with `k` above
    /// the cache capacity always scan directly.
    fn row_mean_distance(&mut self, row: usize, k: usize) -> f32 {
        if k == 0 {
            return f32::INFINITY;
        }
        if k > NEIGHBOR_CACHE_CAPACITY {
            return self.mean_distance_uncached(row, k);
        }
        if (self.neighbor_cache_len[row] as usize) < k {
            self.rebuild_row_cache(row);
        }
        let cache = &self.neighbor_cache[row];
        (0..k)
            .rev()
            .fold(0., |acc, i| acc + cache[i].squared_distance.sqrt())
            / k as f32
    }

    /// Direct O(N) computation of a row's mean distance to its `k` nearest
    /// other rows, used when `k` exceeds the neighbor cache capacity.
    fn mean_distance_uncached(&self, row: usize, k: usize) -> f32 {
        let mut squared_dists = self.squared_distances_to(self.sample(row), N);
        // Skip the self-entry by index instead of dropping the smallest value.
        squared_dists[row] = f32::INFINITY;
        Self::mean_of_smallest(&mut squared_dists, k)
    }

    /// Remaps cached neighbor row indices through `index_map` (`u32::MAX` =
    /// expired) after expiry compaction, shrinking trusted prefixes that
    /// referenced expired rows. Slots at and beyond the retained count keep
    /// stale values; they are reset on append before they can be read again.
    fn remap_neighbor_cache(&mut self, index_map: &[u32; N]) {
        for old_index in 0..N {
            let new_index = index_map[old_index];
            if new_index == u32::MAX {
                continue;
            }
            let mut cache = self.neighbor_cache[old_index];
            let count = self.neighbor_cache_len[old_index] as usize;
            let mut retained = 0;
            for i in 0..count {
                let entry = cache[i];
                let mapped = index_map[entry.row as usize];
                if mapped != u32::MAX {
                    cache[retained] = NeighborEntry {
                        squared_distance: entry.squared_distance,
                        row: mapped,
                    };
                    retained += 1;
                }
            }
            self.neighbor_cache[new_index as usize] = cache;
            self.neighbor_cache_len[new_index as usize] = retained as u8;
        }
    }

    /// Returns index of the buffered row with the lowest mean distance to its
    /// `k` nearest neighbors, derived from the incremental neighbor cache.
    /// Is used when replacing the least useful value in the array.
    fn lowest_mean_distance_by_index(&mut self) -> (usize, f32) {
        let k = self.k.min(N.saturating_sub(1));
        let mut mean_dist: [f32; N] = [0.; N];
        for (i, mean) in mean_dist.iter_mut().enumerate() {
            *mean = self.row_mean_distance(i, k);
        }

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

    /// Add a sample if it is deemed more useful than the least useful sample.
    ///
    /// `gravity_direction` is an optional co-timestamped body-frame FRD
    /// direction. Non-finite and zero directions are ignored. The live quality
    /// and publication state are updated even when diversity rejects the valid
    /// current observation.
    pub fn evaluate_sample_vec(
        &mut self,
        x: Vector3<f32>,
        gravity_direction: Option<Vector3<f32>>,
        timestamp_us: u64,
    ) {
        let valid_current_sample = self.ingest_sample(x, gravity_direction, timestamp_us);
        self.update_publication(valid_current_sample.then_some(x));
    }

    /// Updates the cache and online optimizer, returning whether the current
    /// magnetometer observation was finite and nonzero.
    fn ingest_sample(
        &mut self,
        x: Vector3<f32>,
        gravity_direction: Option<Vector3<f32>>,
        timestamp_us: u64,
    ) -> bool {
        let previous_matrix_filled = self.matrix_filled;
        let mut index_map = [u32::MAX; N];
        let mut retained = 0;
        for (index, map_slot) in index_map.iter_mut().enumerate().take(self.matrix_filled) {
            let sample = self.sample(index);
            if timestamp_us.saturating_sub(self.sample_timestamps_us[index])
                <= self.max_sample_lifespan_us
            {
                *map_slot = retained as u32;
                if retained != index {
                    for column in 0..3 {
                        self.matrix[(retained, column)] = self.matrix[(index, column)];
                    }
                    self.gravity_directions[retained] = self.gravity_directions[index];
                    self.sample_timestamps_us[retained] = self.sample_timestamps_us[index];
                }
                retained += 1;
            } else {
                self.remove_raw_moment(sample);
            }
        }
        if retained != self.matrix_filled {
            self.matrix_filled = retained;
            if retained == 0 {
                // Incremental subtraction can leave round-off residue after
                // the last retained row expires. An empty cache has exact
                // zero moments by definition.
                self.clear_raw_moments();
            }
            self.mean_distance = 0.0;
            self.remap_neighbor_cache(&index_map);
        }
        let expired = retained != previous_matrix_filled;

        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
            if expired {
                self.refresh_normalization();
            }
            return false;
        }
        let gravity_direction = gravity_direction.and_then(|gravity| {
            let norm = gravity.norm();
            if norm.is_finite()
                && gravity.iter().all(|value| value.is_finite())
                && norm > f32::EPSILON
            {
                Some(gravity / norm)
            } else {
                None
            }
        });
        if N == 0 {
            return false;
        }
        let mut accepted_row = None;
        // Check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            let count = self.matrix_filled;
            let squared_dists = self.squared_distances_to(x, count);
            for ((cache, len), &squared_distance) in self
                .neighbor_cache
                .iter_mut()
                .zip(self.neighbor_cache_len.iter_mut())
                .zip(squared_dists.iter())
                .take(count)
            {
                // The cache covers every other row only if it was built up
                // without ever hitting the capacity or losing entries.
                let complete = *len as usize == count - 1;
                Self::cache_insert(
                    cache,
                    len,
                    NeighborEntry {
                        squared_distance,
                        row: count as u32,
                    },
                    complete,
                );
            }
            self.add_raw_moment(x);
            self.add_sample_at(count, x, gravity_direction, timestamp_us);
            self.reset_row_cache(count, &squared_dists, count);
            self.matrix_filled += 1;
            accepted_row = Some(count);
        }
        // Otherwise check which sample may be best to replace
        else {
            let k = self.k.min(N.saturating_sub(1));
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let squared_dists = self.squared_distances_to(x, N);
            // The candidate has no self-entry in the buffer, so its mean
            // distance covers the true k nearest buffered rows.
            let mut scratch = squared_dists;
            let sample_mean_dist = Self::mean_of_smallest(&mut scratch, k);
            if low_mean_dist < sample_mean_dist {
                for (row, ((cache, len), &squared_distance)) in self
                    .neighbor_cache
                    .iter_mut()
                    .zip(self.neighbor_cache_len.iter_mut())
                    .zip(squared_dists.iter())
                    .enumerate()
                {
                    if row == low_index {
                        continue;
                    }
                    Self::cache_remove(cache, len, low_index as u32);
                    // After removal the cache covers every row besides the
                    // row itself and the replaced one only if nothing was
                    // ever evicted from it.
                    let complete = *len as usize == N.saturating_sub(2);
                    Self::cache_insert(
                        cache,
                        len,
                        NeighborEntry {
                            squared_distance,
                            row: low_index as u32,
                        },
                        complete,
                    );
                }
                self.remove_raw_moment(self.sample(low_index));
                self.add_raw_moment(x);
                self.add_sample_at(low_index, x, gravity_direction, timestamp_us);
                self.reset_row_cache(low_index, &squared_dists, N);
                accepted_row = Some(low_index);
            }
        }
        if expired || accepted_row.is_some() {
            self.refresh_normalization();
        }
        self.update_online_optimizer(x, gravity_direction, accepted_row);
        true
    }

    /// Insert a sample vector into `index` row of buffer matrix.
    fn add_sample_at(
        &mut self,
        index: usize,
        sample: Vector3<f32>,
        gravity_direction: Option<Vector3<f32>>,
        timestamp_us: u64,
    ) {
        if index < N {
            self.matrix[(index, 0)] = sample[0];
            self.matrix[(index, 1)] = sample[1];
            self.matrix[(index, 2)] = sample[2];
            self.gravity_directions[index] = gravity_direction;
            self.sample_timestamps_us[index] = timestamp_us;
        }
    }

    /// Quadratic feature vector of a unit direction: the nine ellipsoid-fit
    /// features with `sqrt(2)` cross-term weights. With this weighting the
    /// feature norm equals the rotation-invariant `tr(d d^T d d^T)`, so the
    /// induced rotation on feature space is orthogonal and the design
    /// eigenvalues are exactly rotation-invariant. Under the uniform
    /// spherical distribution `E[phi phi^T]` has eigenvalues `{1/3 x4, 2/15
    /// x5}`.
    fn direction_feature(d: Vector3<f32>) -> SVector<f32, CALIBRATION_PARAMETER_COUNT> {
        SVector::<f32, CALIBRATION_PARAMETER_COUNT>::from_column_slice(&[
            d.x * d.x,
            d.y * d.y,
            d.z * d.z,
            std::f32::consts::SQRT_2 * d.x * d.y,
            std::f32::consts::SQRT_2 * d.x * d.z,
            std::f32::consts::SQRT_2 * d.y * d.z,
            d.x,
            d.y,
            d.z,
        ])
    }

    /// E-optimality coverage of the retained directions: the smallest
    /// eigenvalue of the mean design matrix relative to the uniform-sphere
    /// reference. Rotation-invariant by construction, and a cache whose
    /// directions support fewer than nine independent features (for example
    /// near-planar motion) is rank-deficient and scores near zero.
    fn coverage_from_design(design_matrix: &DesignMatrix, matrix_filled: usize) -> f32 {
        if matrix_filled < CALIBRATION_PARAMETER_COUNT {
            return 0.0;
        }
        let mean_design = design_matrix / matrix_filled as f32;
        let lambda_min = SymmetricEigen::new(mean_design).eigenvalues.min();
        (lambda_min / COVERAGE_LAMBDA_REF).clamp(0.0, 1.0)
    }

    /// Coverage of the retained rows, mean-centered and recomputed from the
    /// current cache on each quality update. Recomputing keeps every
    /// direction centered on the current cache mean, so no insertion-time
    /// snapshots, incremental design state, or drift-triggered rebuilds are
    /// needed. The cache mean is used rather than the fitted hard-iron
    /// offset: the offset's component along the thinnest data direction is
    /// itself unconstrained for near-planar support, which destabilizes the
    /// score exactly where it must be decisive. A near-planar cache stays
    /// rank-deficient under any centering.
    fn mean_centered_coverage(&self) -> f32 {
        let mut design = DesignMatrix::zeros();
        for row in 0..self.matrix_filled {
            let centered = self.sample(row) - self.normalization_mean;
            let norm = centered.norm();
            if norm.is_finite() && norm > f32::EPSILON {
                let phi = Self::direction_feature(centered / norm);
                design += phi * phi.transpose();
            }
        }
        Self::coverage_from_design(&design, self.matrix_filled)
    }

    /// Get mean distance value between samples in matrix buffer.
    pub fn get_mean_distance(&self) -> f32 {
        self.mean_distance
    }

    /// Returns the current bounded calibration quality in `[0, 1]`.
    ///
    /// Zero means the current working candidate is pending or unusable. A
    /// previously published correction can remain available while this value
    /// is zero after a rejected later candidate.
    pub fn get_confidence(&self) -> f32 {
        self.confidence
    }

    /// Calibrates a magnetometer vector that has already been converted to FRD.
    ///
    /// `gravity_direction` is an optional co-timestamped body-frame FRD
    /// direction. It contributes a convex constant-projection surrogate to the
    /// online ellipsoid fit without making gravity mandatory for calibration.
    pub fn evaluate_correct(
        &mut self,
        raw_mag: Vector3<f32>,
        gravity_direction: Option<Vector3<f32>>,
        timestamp_us: u64,
    ) -> Result<MagCalibrationResult, BadMagCause> {
        self.evaluate_sample_vec(raw_mag, gravity_direction, timestamp_us);
        if !self.calibration_initialized {
            return Ok(MagCalibrationResult {
                confidence: self.confidence,
                direction: None,
            });
        }
        let mag = self.soft_iron_correction * (raw_mag - self.hard_iron_offset);

        let mag_norm = mag.norm();
        if !mag_norm.is_finite() || mag_norm < MIN_MAG_NORM {
            Err(BadMagCause::BadReading(BadReading::WeakCalibratedReading {
                norm: mag_norm,
                min_norm: MIN_MAG_NORM,
            }))
        } else {
            Ok(MagCalibrationResult {
                confidence: self.confidence,
                direction: Some(mag.normalize()),
            })
        }
    }

    fn update_publication(&mut self, current_sample: Option<Vector3<f32>>) {
        let current_sample_valid = current_sample.is_some();
        let candidate = self.update_quality(current_sample);
        if !current_sample_valid || candidate.is_none() {
            // Invalid observations and unusable candidates always reset the
            // streak: they are evidence against publishing, not jitter.
            self.publication_quality_streak = 0;
        } else if self.confidence >= MIN_PUBLICATION_CONFIDENCE {
            self.publication_quality_streak = self.publication_quality_streak.saturating_add(1);
        } else if self.confidence < PUBLICATION_STREAK_RESET_CONFIDENCE {
            // Only a genuine quality collapse restarts the streak; a short
            // dip in live quality while the optimizer absorbs newly visited
            // directions merely pauses it.
            self.publication_quality_streak = 0;
        }
        if self.publication_quality_streak >= MIN_PUBLICATION_STREAK {
            if let Some(candidate) = candidate {
                self.hard_iron_offset = candidate.offset;
                self.soft_iron_correction = candidate.correction;
                self.calibration_initialized = true;
            }
        }
    }

    /// Derives one finite SPD correction candidate from the current online
    /// ellipsoid state without scanning retained rows.
    fn working_candidate(&self) -> Result<CalibrationCandidate, BadCalibration> {
        if !self.normalization_initialized
            || !self
                .normalization_mean
                .iter()
                .all(|value| value.is_finite())
            || !self.normalization_radius.is_finite()
            || self.normalization_radius <= f32::EPSILON
        {
            return Err(BadCalibration::Unsolveable {
                message: "sample normalization is non-finite or zero",
            });
        }
        let parameters = self.parameters;
        if !parameters.iter().all(|value| value.is_finite()) {
            return Err(BadCalibration::Unsolveable {
                message: "online calibration produced non-finite parameters",
            });
        }

        let (shape, linear) = Self::shape_and_linear(&parameters);
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
            shape_eigen.eigenvectors * square_root * shape_eigen.eigenvectors.transpose()
                / self.normalization_radius;
        let offset = self.normalization_mean + self.normalization_radius * normalized_offset;
        if !offset.iter().all(|value| value.is_finite())
            || !correction.iter().all(|value| value.is_finite())
        {
            return Err(BadCalibration::Unsolveable {
                message: "calibration produced non-finite parameters",
            });
        }

        Ok(CalibrationCandidate { offset, correction })
    }

    fn update_running_mean_square(
        current: Option<f32>,
        residual_squared: f32,
        alpha: f32,
    ) -> Option<f32> {
        if !residual_squared.is_finite()
            || residual_squared < 0.0
            || !alpha.is_finite()
            || !(0.0..=1.0).contains(&alpha)
            || alpha == 0.0
        {
            return None;
        }
        let current = current.unwrap_or(0.0);
        if !current.is_finite() || current < 0.0 {
            return None;
        }
        let updated = current + alpha * (residual_squared - current);
        updated.is_finite().then_some(updated.max(0.0))
    }

    fn fitness_score(mean_square: Option<f32>) -> f32 {
        match mean_square {
            Some(mean_square) if mean_square.is_finite() && mean_square >= 0.0 => {
                (1.0 - mean_square.sqrt() / MAX_RADIAL_RMS).clamp(0.0, 1.0)
            }
            _ => 0.0,
        }
    }

    /// Updates the live radial statistic and quality for the current working
    /// candidate. All cache-dependent data comes from maintained moments.
    fn update_quality(
        &mut self,
        current_sample: Option<Vector3<f32>>,
    ) -> Option<CalibrationCandidate> {
        if self.matrix_filled < CALIBRATION_PARAMETER_COUNT {
            self.confidence = 0.0;
            return None;
        }
        let candidate = match self.working_candidate() {
            Ok(candidate) => candidate,
            Err(_) => {
                self.confidence = 0.0;
                return None;
            }
        };
        if let Some(sample) = current_sample {
            let residual = (candidate.correction * (sample - candidate.offset)).norm() - 1.0;
            let residual_squared = residual * residual;
            let alpha = 1.0 / self.matrix_filled.min(self.minibatch_size).max(1) as f32;
            let Some(mean_square) = Self::update_running_mean_square(
                self.radial_residual_mean_square,
                residual_squared,
                alpha,
            ) else {
                self.radial_residual_mean_square = None;
                self.confidence = 0.0;
                return None;
            };
            self.radial_residual_mean_square = Some(mean_square);
        }
        let coverage = self.mean_centered_coverage();
        let fitness = Self::fitness_score(self.radial_residual_mean_square);
        let quality = coverage * fitness;
        self.confidence = if quality.is_finite() {
            quality.clamp(0.0, 1.0)
        } else {
            0.0
        };
        Some(candidate)
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

    #[cfg(test)]
    pub(super) fn working_quality_components(&self) -> (bool, f32, f32) {
        let Ok(_candidate) = self.working_candidate() else {
            return (false, 0.0, 0.0);
        };
        let coverage = self.mean_centered_coverage();
        let fitness = Self::fitness_score(self.radial_residual_mean_square);
        (true, coverage, fitness)
    }

    #[cfg(test)]
    pub(super) fn correct_working_for_test(&self, raw_mag: Vector3<f32>) -> Option<Vector3<f32>> {
        let candidate = self.working_candidate().ok()?;
        let corrected = candidate.correction * (raw_mag - candidate.offset);
        let norm = corrected.norm();
        (norm.is_finite() && norm > f32::EPSILON).then(|| corrected / norm)
    }

    #[cfg(test)]
    pub(super) fn publication_quality_streak_for_test(&self) -> usize {
        self.publication_quality_streak
    }

    #[cfg(test)]
    pub(super) fn coverage_scores_for_test(
        design_matrix: &DesignMatrix,
        matrix_filled: usize,
    ) -> f32 {
        Self::coverage_from_design(design_matrix, matrix_filled)
    }

    #[cfg(test)]
    pub(super) fn design_matrix_for_test(directions: &[Vector3<f32>]) -> DesignMatrix {
        let mut design = DesignMatrix::zeros();
        for &direction in directions {
            let phi = Self::direction_feature(direction);
            design += phi * phi.transpose();
        }
        design
    }

    #[cfg(test)]
    pub(super) fn fitness_score_for_test(mean_square: Option<f32>) -> f32 {
        Self::fitness_score(mean_square)
    }

    #[cfg(test)]
    pub(super) fn running_mean_square_for_test(
        current: Option<f32>,
        residual_squared: f32,
        alpha: f32,
    ) -> Option<f32> {
        Self::update_running_mean_square(current, residual_squared, alpha)
    }

    #[cfg(test)]
    pub(super) fn radial_residual_mean_square_for_test(&self) -> Option<f32> {
        self.radial_residual_mean_square
    }

    #[cfg(test)]
    pub(super) fn raw_moments_for_test(&self) -> (usize, Vector3<f64>, Matrix3<f64>) {
        (
            self.matrix_filled,
            self.raw_sample_sum,
            self.raw_outer_product_sum,
        )
    }

    /// Verifies maintained raw moments against a direct current-cache sum.
    #[cfg(test)]
    pub(super) fn check_raw_moments(&self) -> Result<(), String> {
        let (sum, outer_sum) = (0..self.matrix_filled).fold(
            (Vector3::<f64>::zeros(), Matrix3::<f64>::zeros()),
            |(sum, outer_sum), row| {
                let sample = self.sample(row).cast::<f64>();
                (sum + sample, outer_sum + sample * sample.transpose())
            },
        );
        let scale = sum.norm().max(outer_sum.norm()).max(1.0);
        let error = (self.raw_sample_sum - sum)
            .norm()
            .max((self.raw_outer_product_sum - outer_sum).norm());
        if error <= 1.0e-12 * scale {
            Ok(())
        } else {
            Err(format!("raw moment error={error} scale={scale}"))
        }
    }

    /// Verifies the neighbor-cache invariant against the current buffer
    /// contents: for every buffered row, the cached entries must reference
    /// distinct live rows with exactly matching squared distances, be sorted
    /// ascending, and their distance values must equal the `len` smallest
    /// true distances to the row's other buffered rows. Read-only; used by
    /// tests to cross-check the incremental cache maintenance.
    #[cfg(test)]
    pub(super) fn check_neighbor_cache(&self) -> Result<(), String> {
        for row in 0..self.matrix_filled {
            let len = self.neighbor_cache_len[row] as usize;
            let cache = &self.neighbor_cache[row][..len];
            let mut true_dists: Vec<f32> = (0..self.matrix_filled)
                .filter(|&j| j != row)
                .map(|j| {
                    let diff = self.sample(row) - self.sample(j);
                    diff.dot(&diff)
                })
                .collect();
            true_dists.sort_unstable_by(|a, b| a.total_cmp(b));
            for (i, entry) in cache.iter().enumerate() {
                if entry.row as usize >= self.matrix_filled || entry.row as usize == row {
                    return Err(format!("row {row}: entry {i} references row {}", entry.row));
                }
                if i > 0 && cache[i - 1].squared_distance > entry.squared_distance {
                    return Err(format!("row {row}: entry {i} out of order"));
                }
                if cache[..i].iter().any(|e| e.row == entry.row) {
                    return Err(format!("row {row}: duplicate entry for row {}", entry.row));
                }
                let diff = self.sample(row) - self.sample(entry.row as usize);
                if diff.dot(&diff) != entry.squared_distance {
                    return Err(format!("row {row}: stale distance for row {}", entry.row));
                }
                if true_dists.get(i) != cache.get(i).map(|e| &e.squared_distance) {
                    return Err(format!(
                        "row {row}: entry {i} is not the true {}-nearest neighbor",
                        i + 1
                    ));
                }
            }
            if len > true_dists.len() {
                return Err(format!("row {row}: cache longer than the neighbor pool"));
            }
        }
        Ok(())
    }
}
