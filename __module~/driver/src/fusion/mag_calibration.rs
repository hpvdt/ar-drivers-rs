// use core::cmp::Ordering;
use nalgebra::{DMatrix, DVector, SMatrix, SMatrixView, Vector3, SVD};

use super::BadMagDataCause;

const DESIGN_MATRIX_COLUMNS: usize = 6;
const SVD_EPSILON_RATIO: f32 = 1.0e-6;
const MAX_SVD_CONDITION: f32 = 1.0e6;

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
    matrix_filled: usize,
    mean_distance: f32,
    pre_scaler: f32,
    k: usize,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::from_element(1.0),
            matrix_filled: Default::default(),
            mean_distance: Default::default(),
            pre_scaler: 1.,
            k: 2, // Works well in testing
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
    pub fn evaluate_sample(&mut self, x: [f32; 3]) {
        self.evaluate_sample_vec(Vector3::from(x))
    }

    /// Add a sample if it is deemed more useful than the least useful sample.
    pub fn evaluate_sample_vec(&mut self, x: Vector3<f32>) {
        if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
            return;
        }
        // Check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            self.add_sample_at(self.matrix_filled, x);
            self.matrix_filled += 1;
        }
        // Otherwise check which sample may be best to replace
        else {
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let sample_mean_dist = self.mean_distance_from_single(x.transpose());
            if low_mean_dist < sample_mean_dist {
                self.add_sample_at(low_index, x);
            }
        }
    }

    /// Insert a sample vector into `index` row of buffer matrix.
    fn add_sample_at(&mut self, index: usize, sample: Vector3<f32>) {
        if index < N {
            self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
            self.matrix[(index, 1)] = sample[1] / self.pre_scaler;
            self.matrix[(index, 2)] = sample[2] / self.pre_scaler;
        }
    }

    /// Get mean distance value between samples in matrix buffer.
    pub fn get_mean_distance(&self) -> f32 {
        self.mean_distance
    }

    /// Try to calculate calibration offset and scale values. Returns the cause
    /// when the calibration cannot be produced. The tuple contains (offset, scale).
    pub fn perform_calibration(
        &mut self,
    ) -> std::result::Result<([f32; 3], [f32; 3]), BadMagDataCause> {
        let sample_count = self.matrix_filled.min(N);
        if sample_count < DESIGN_MATRIX_COLUMNS {
            return Err(BadMagDataCause::InsufficientCalibrationSamples {
                samples: sample_count,
                required: DESIGN_MATRIX_COLUMNS,
            });
        }

        // Calculate column 4 and 5 of H matrix for the real samples only.
        self.matrix
            .row_iter_mut()
            .take(sample_count)
            .for_each(|mut mag| {
                mag[3] = -mag[1] * mag[1];
                mag[4] = -mag[2] * mag[2];
            });

        let design = DMatrix::from_fn(sample_count, DESIGN_MATRIX_COLUMNS, |row, col| {
            self.matrix[(row, col)]
        });
        let w = DVector::from_fn(sample_count, |row, _| {
            self.matrix[(row, 0)] * self.matrix[(row, 0)]
        });

        let svd = SVD::new(design, true, true);
        let singular_values = svd.singular_values.as_slice();
        let max_singular_value = singular_values[0];
        let min_singular_value = singular_values[DESIGN_MATRIX_COLUMNS - 1];
        let epsilon = max_singular_value * SVD_EPSILON_RATIO;

        let rank = svd.rank(epsilon);
        if !max_singular_value.is_finite()
            || max_singular_value <= 0.0
            || rank < DESIGN_MATRIX_COLUMNS
        {
            return Err(BadMagDataCause::DegenerateCalibrationSamples {
                rank,
                required_rank: DESIGN_MATRIX_COLUMNS,
            });
        }

        let condition = max_singular_value / min_singular_value;
        if !condition.is_finite() || condition > MAX_SVD_CONDITION {
            return Err(BadMagDataCause::IllConditionedCalibrationSamples {
                condition,
                max_condition: MAX_SVD_CONDITION,
            });
        }

        // Solve the least-squares system with a Moore-Penrose pseudo-inverse.
        let pseudo_inverse = svd
            .pseudo_inverse(epsilon)
            .map_err(|message| BadMagDataCause::CalibrationSolveFailed { message })?;
        let x = pseudo_inverse * w;

        // Calculate offsets and scale factors in pre-scaled sample units.
        let offset = Vector3::new(x[0] / 2., x[1] / (2. * x[3]), x[2] / (2. * x[4]));
        let temp = x[5] + offset[0] * offset[0] + x[3] * offset[1] * offset[1] + x[4] * offset[2] * offset[2];
        let scale = Vector3::new(temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt());

        // Samples are divided by `pre_scaler` before storage, so the fit is in
        // pre-scaled units. Convert offset and scale back to caller-visible (raw)
        // units so `(raw - offset) / scale` operates in the same units as the input.
        let offset = offset * self.pre_scaler;
        let scale = scale * self.pre_scaler;

        // Check that off and scale vectors contain valid values
        for component in offset.iter().chain(scale.iter()) {
            if !component.is_finite() {
                return Err(BadMagDataCause::NonFiniteCalibration { offset, scale });
            }
        }

        // TODO Add option for low-pass filtering this result
        Ok((offset.into(), scale.into()))
    }
}
