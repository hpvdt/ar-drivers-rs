# Mag Calibration Algorithm Job

## Tasks

- [ ] Accept valid samples with zero components
  - `evaluate_sample_vec` currently rejects any component that is not `is_normal()`.
  - Replace this with a check that rejects `NaN`/infinite values and near-zero field vectors, while allowing valid finite zero components.
  - Add coverage for samples like `[45.0, 0.0, -12.0]`.

- [ ] Prevent calibration before enough real samples are collected
  - `perform_calibration` can currently solve using default-filled rows before the buffer has `N` measurements.
  - Return `None` while `matrix_filled < N`, or solve only over the filled rows.
  - Add a test proving unfilled default rows do not affect calibration.

- [ ] Return calibration values in caller-visible units
  - Samples are divided by `pre_scaler` before storage, but returned offsets and scales are not converted back.
  - Define the intended unit contract for `pre_scaler` and adjust returned offset/scale values accordingly.
  - Add a test that changing `pre_scaler` does not silently change physical calibration units.

- [ ] Fix k-nearest-neighbor distance calculation
  - `mean_distance_from_single` includes self-distance for existing rows, excludes it for new samples, and divides by `N` instead of the neighbor count.
  - Compute the mean over exactly `k` comparable neighbors, with consistent handling for existing and candidate samples.
  - Clamp or reject invalid `k` values so the method has defined behavior.

- [x] Use a numerically robust least-squares solver
  - `perform_calibration` describes pseudo-inverse but uses normal equations and `try_inverse`.
  - Replace this with a more stable QR/SVD-based least-squares solve and detect rank-deficient or poorly conditioned data.
  - Add coverage for degenerate sample distributions returning `None`.
  - Addressed with an SVD solve plus rank/conditioning check. `A^T A` is always positive-semidefinite, but PSD still permits singular or poorly conditioned matrices.
