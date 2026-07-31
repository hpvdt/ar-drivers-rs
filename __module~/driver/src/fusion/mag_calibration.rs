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
const MAX_REFINEMENT_RADIAL_RMS_INCREASE: f32 = 0.005;
const REFINEMENT_DAMPING: f32 = 1.0e-2;
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

/// Direct regularized ellipsoid fit for a hard-iron offset and full SPD
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
            gravity_weight: 0.1,
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
    /// The default is 0.1; zero disables gravity refinement.
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

        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
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
            }
        }
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
    /// direction. It refines the fit using the constant magnetic dip without
    /// making gravity mandatory for calibration.
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
        // Regularize toward a scaled identity shape rather than toward the
        // (non-PD) zero matrix. The quadratic part is unchanged; the linear
        // term moves to the right-hand side, keeping a single direct solve.
        for (index, weight) in [1.0, 1.0, 1.0, 2.0, 2.0, 2.0].into_iter().enumerate() {
            normal[(index, index)] += SHAPE_REGULARIZATION * weight;
        }
        for index in 0..3 {
            right_hand_side[index] += SHAPE_REGULARIZATION * SHAPE_PRIOR_SCALE;
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

        let (correction, offset) =
            self.refine_with_gravity(correction, offset, sample_mean, radius, sample_count);

        self.hard_iron_offset = offset;
        self.soft_iron_correction = correction;
        self.calibration_initialized = true;
        Ok(())
    }

    /// Applies one damped quadratic refinement to the validated ellipsoid fit.
    /// The unknowns are the six elements of the symmetric correction and the
    /// three elements of its affine offset in normalized sample coordinates.
    fn refine_with_gravity(
        &self,
        correction: Matrix3<f32>,
        offset: Vector3<f32>,
        sample_mean: Vector3<f32>,
        radius: f32,
        sample_count: usize,
    ) -> (Matrix3<f32>, Vector3<f32>) {
        let gravity_count = (0..sample_count)
            .filter(|&row| self.gravity_directions[row].is_some())
            .count();
        if gravity_count < 2 || self.gravity_weight == 0.0 {
            return (correction, offset);
        }

        let normalized_correction = correction * radius;
        let affine_offset = correction * (sample_mean - offset);
        let mut gravity_mean = 0.0;
        let mut gravity_jacobian_mean = SVector::<f32, 9>::zeros();
        for row in 0..sample_count {
            let Some(gravity) = self.gravity_directions[row] else {
                continue;
            };
            let sample = (self.sample(row) - sample_mean) / radius;
            let basis = Self::affine_basis(sample);
            gravity_mean += gravity.dot(&(normalized_correction * sample + affine_offset));
            gravity_jacobian_mean += basis.transpose() * gravity;
        }
        gravity_mean /= gravity_count as f32;
        gravity_jacobian_mean /= gravity_count as f32;

        let mut normal = SMatrix::<f32, 9, 9>::zeros();
        let mut right_hand_side = SVector::<f32, 9>::zeros();
        for row in 0..sample_count {
            let sample = (self.sample(row) - sample_mean) / radius;
            let basis = Self::affine_basis(sample);
            let calibrated = normalized_correction * sample + affine_offset;
            let norm = calibrated.norm();
            if !norm.is_finite() || norm <= f32::EPSILON {
                return (correction, offset);
            }

            let radial_jacobian = basis.transpose() * (calibrated / norm);
            let radial_residual = norm - 1.0;
            normal += radial_jacobian * radial_jacobian.transpose() / sample_count as f32;
            right_hand_side -= radial_jacobian * radial_residual / sample_count as f32;

            if let Some(gravity) = self.gravity_directions[row] {
                let gravity_jacobian = basis.transpose() * gravity - gravity_jacobian_mean;
                let gravity_residual = gravity.dot(&calibrated) - gravity_mean;
                normal += self.gravity_weight * gravity_jacobian * gravity_jacobian.transpose()
                    / gravity_count as f32;
                right_hand_side -= self.gravity_weight * gravity_jacobian * gravity_residual
                    / gravity_count as f32;
            }
        }
        for index in 0..9 {
            normal[(index, index)] += REFINEMENT_DAMPING * normal[(index, index)].max(f32::EPSILON);
        }

        let Some(cholesky) = normal.cholesky() else {
            return (correction, offset);
        };
        let delta = cholesky.solve(&right_hand_side);
        if !delta.iter().all(|value| value.is_finite()) {
            return (correction, offset);
        }

        let delta_correction = Matrix3::new(
            delta[0], delta[3], delta[4], delta[3], delta[1], delta[5], delta[4], delta[5],
            delta[2],
        );
        let delta_offset = Vector3::new(delta[6], delta[7], delta[8]);
        let Some((candidate_objective, candidate_radial_rms)) = self.refinement_objective(
            &normalized_correction,
            &affine_offset,
            sample_mean,
            radius,
            sample_count,
        ) else {
            return (correction, offset);
        };

        let mut step = 1.0;
        for _ in 0..4 {
            let refined_correction = normalized_correction + step * delta_correction;
            let refined_affine_offset = affine_offset + step * delta_offset;
            if !refined_correction.iter().all(|value| value.is_finite())
                || !refined_affine_offset.iter().all(|value| value.is_finite())
            {
                step *= 0.5;
                continue;
            }

            let Some(refined_cholesky) = refined_correction.cholesky() else {
                step *= 0.5;
                continue;
            };
            let refined_condition =
                Self::condition_number(&refined_correction.symmetric_eigen().eigenvalues);
            if refined_condition > MAX_CORRECTION_CONDITION {
                step *= 0.5;
                continue;
            }
            let Some((objective, radial_rms)) = self.refinement_objective(
                &refined_correction,
                &refined_affine_offset,
                sample_mean,
                radius,
                sample_count,
            ) else {
                step *= 0.5;
                continue;
            };
            if radial_rms > MAX_RADIAL_RMS
                || radial_rms > candidate_radial_rms + MAX_REFINEMENT_RADIAL_RMS_INCREASE
                || objective >= candidate_objective
            {
                step *= 0.5;
                continue;
            }

            let refined_offset =
                sample_mean - radius * refined_cholesky.solve(&refined_affine_offset);
            return (refined_correction / radius, refined_offset);
        }
        (correction, offset)
    }

    fn refinement_objective(
        &self,
        correction: &Matrix3<f32>,
        affine_offset: &Vector3<f32>,
        sample_mean: Vector3<f32>,
        radius: f32,
        sample_count: usize,
    ) -> Option<(f32, f32)> {
        let mut radial_squared = 0.0;
        let mut gravity_count = 0;
        let mut gravity_mean = 0.0;
        for row in 0..sample_count {
            let sample = (self.sample(row) - sample_mean) / radius;
            let calibrated = correction * sample + affine_offset;
            let radial_residual = calibrated.norm() - 1.0;
            radial_squared += radial_residual * radial_residual;
            if let Some(gravity) = self.gravity_directions[row] {
                gravity_mean += gravity.dot(&calibrated);
                gravity_count += 1;
            }
        }
        gravity_mean /= gravity_count as f32;

        let mut gravity_squared = 0.0;
        for row in 0..sample_count {
            if let Some(gravity) = self.gravity_directions[row] {
                let sample = (self.sample(row) - sample_mean) / radius;
                let calibrated = correction * sample + affine_offset;
                let residual = gravity.dot(&calibrated) - gravity_mean;
                gravity_squared += residual * residual;
            }
        }
        let radial_mean = radial_squared / sample_count as f32;
        let objective = radial_mean + self.gravity_weight * gravity_squared / gravity_count as f32;
        if objective.is_finite() {
            Some((objective, radial_mean.sqrt()))
        } else {
            None
        }
    }

    fn affine_basis(sample: Vector3<f32>) -> SMatrix<f32, 3, 9> {
        SMatrix::from_row_slice(&[
            sample.x, 0.0, 0.0, sample.y, sample.z, 0.0, 1.0, 0.0, 0.0, 0.0, sample.y, 0.0,
            sample.x, 0.0, sample.z, 0.0, 1.0, 0.0, 0.0, 0.0, sample.z, 0.0, sample.x, sample.y,
            0.0, 0.0, 1.0,
        ])
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
