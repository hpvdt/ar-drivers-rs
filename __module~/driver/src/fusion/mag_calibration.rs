use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use super::bad_mag_cause::{BadCalibration, BadMagCause, BadReading};

const CALIBRATION_PARAMETER_COUNT: usize = 9;
const SHAPE_REGULARIZATION: f32 = 1.0e-3;
/// Scale of the regularization target shape, in units of the identity.
/// Algebraic ellipsoid fits under noise systematically inflate the ellipsoid
/// (underestimate the eigenvalues of the shape matrix), so the prior centers
/// on a shape larger than the ideal sphere to counter that bias.
const SHAPE_PRIOR_SCALE: f32 = 2.0;
const MAX_SAMPLE_CONDITION: f32 = 1.0e2;
const MAX_CORRECTION_CONDITION: f32 = 1.0e1;
const MAX_RADIAL_RMS: f32 = 0.1;
const MIN_MAG_NORM: f32 = 0.4;
const DEFAULT_GRAVITY_WEIGHT: f32 = 0.01;
const DEFAULT_MINIBATCH_SIZE: usize = 32;
const ONLINE_INITIAL_LEARNING_RATE: f32 = 0.5;
const ONLINE_LEARNING_RATE_DECAY_STEPS: f32 = 64.0;
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
    prng_state: u64,
    optimizer_steps: u64,
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
            prng_state: ONLINE_PRNG_SEED,
            optimizer_steps: 0,
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
    }

    /// Recomputes the current cache normalization and analytically transforms
    /// the working quadratic equation into it. A failed transform resets only
    /// unpublished optimizer state.
    fn refresh_normalization(&mut self) {
        let sample_count = self.matrix_filled.min(N);
        if sample_count == 0 {
            self.normalization_mean = Vector3::zeros();
            self.normalization_radius = 0.0;
            self.normalization_initialized = false;
            self.reset_working_state();
            return;
        }

        let sample_mean = (0..sample_count)
            .fold(Vector3::zeros(), |sum, row| sum + self.sample(row))
            / sample_count as f32;
        let radius_squared = (0..sample_count)
            .map(|row| (self.sample(row) - sample_mean).norm_squared())
            .sum::<f32>()
            / sample_count as f32;
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
        current_sample: Vector3<f32>,
        current_gravity: Option<Vector3<f32>>,
        accepted_row: Option<usize>,
        random_draws: usize,
        mut random_state: u64,
    ) -> f32 {
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

        add_observation(current_sample, current_gravity);
        for _ in 0..random_draws {
            let Some(row) =
                Self::random_cache_row(&mut random_state, self.matrix_filled, accepted_row)
            else {
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

    /// Applies one bounded normalized-SGD update to the current valid sample
    /// and randomly selected retained observations.
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
        let random_state = self.prng_state;
        let mut next_random_state = random_state;
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

        add_observation(current_sample, current_gravity);
        for _ in 0..random_draws {
            let Some(row) =
                Self::random_cache_row(&mut next_random_state, self.matrix_filled, accepted_row)
            else {
                break;
            };
            add_observation(self.sample(row), self.gravity_directions[row]);
        }
        self.prng_state = next_random_state;

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
            return;
        }

        let old_objective = self.minibatch_objective(
            &parameters,
            gravity_projection,
            current_sample,
            current_gravity,
            accepted_row,
            random_draws,
            random_state,
        );
        let learning_rate = (ONLINE_INITIAL_LEARNING_RATE
            / (1.0 + self.optimizer_steps as f32 / ONLINE_LEARNING_RATE_DECAY_STEPS))
            .max(ONLINE_MIN_LEARNING_RATE);
        let mut step = learning_rate.min(ONLINE_MAX_STEP_NORM / direction_norm);
        for _ in 0..ONLINE_BACKTRACK_STEPS {
            let candidate = parameters - step * direction;
            let candidate_gravity_projection = gravity_projection - step * gravity_direction;
            let objective = self.minibatch_objective(
                &candidate,
                candidate_gravity_projection,
                current_sample,
                current_gravity,
                accepted_row,
                random_draws,
                random_state,
            );
            if candidate.iter().all(|value| value.is_finite())
                && candidate_gravity_projection.is_finite()
                && objective.is_finite()
                && objective < old_objective
            {
                self.parameters = candidate;
                self.gravity_projection = candidate_gravity_projection;
                self.optimizer_steps += 1;
                return;
            }
            step *= 0.5;
        }
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
    /// direction. Non-finite and zero directions are ignored.
    pub fn evaluate_sample_vec(
        &mut self,
        x: Vector3<f32>,
        gravity_direction: Option<Vector3<f32>>,
        timestamp_us: u64,
    ) {
        let previous_matrix_filled = self.matrix_filled;
        let mut index_map = [u32::MAX; N];
        let mut retained = 0;
        for (index, map_slot) in index_map.iter_mut().enumerate().take(self.matrix_filled) {
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
            }
        }
        if retained != self.matrix_filled {
            self.matrix_filled = retained;
            self.mean_distance = 0.0;
            self.remap_neighbor_cache(&index_map);
        }
        let expired = retained != previous_matrix_filled;

        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
            if expired {
                self.refresh_normalization();
            }
            return;
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
            return;
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
                self.add_sample_at(low_index, x, gravity_direction, timestamp_us);
                self.reset_row_cache(low_index, &squared_dists, N);
                accepted_row = Some(low_index);
            }
        }
        if expired || accepted_row.is_some() {
            self.refresh_normalization();
        }
        self.update_online_optimizer(x, gravity_direction, accepted_row);
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

    /// Get mean distance value between samples in matrix buffer.
    pub fn get_mean_distance(&self) -> f32 {
        self.mean_distance
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
    ) -> Result<Vector3<f32>, BadMagCause> {
        self.evaluate_sample_vec(raw_mag, gravity_direction, timestamp_us);
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

    /// Validates the online ellipsoid estimate and derives its symmetric
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
