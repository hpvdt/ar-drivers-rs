
## Tasks

- [ ] Return calibration values in caller-visible units
    - Samples are divided by `pre_scaler` before storage, but returned offsets and scales are not converted back.
    - Define the intended unit contract for `pre_scaler` and adjust returned offset/scale values accordingly.
    - Add a test that changing `pre_scaler` does not silently change physical calibration units.

- [ ] Replace axis-aligned magnetometer error-state estimation with an SPD full soft-iron model
    - Scope this change to the calibration/error-state solve and the representation it returns. Keep sample validation, buffering, KNN diversity scoring, and eviction behavior in `MagCalibrator` intact.
    - Replace the current `(offset: [f32; 3], scale: [f32; 3])` diagonal model with `(offset: Vector3<f32>, correction: Matrix3<f32>)`, where the correction is derived from a symmetric positive definite ellipsoid shape.
    - Estimate the ellipsoid with 9 parameters: hard-iron offset `b` plus a lower-triangular Cholesky factor `L` with positive diagonal, so `A = L * L.transpose()` is SPD by construction.
    - Minimize the unit-sphere residual `||L.transpose() * (sample - b)||^2 - 1` with Levenberg-Marquardt and an analytic Jacobian. The fixed 9x9 normal-equation solve keeps each iteration linear in the number of buffered samples.
    - Initialize `b` from the buffered sample min/max midpoint or mean, initialize `L` diagonally from per-axis extents, and leave off-diagonal terms at zero before optimization.
    - Apply calibration as `correction * (raw_mag - offset)` and normalize the result for heading use. Do not add a fallback to raw magnetometer data.
    - Replace diagonal scale-specific safety checks with fit validation appropriate for the SPD model: finite parameters, positive Cholesky diagonal, bounded condition number, bounded residual loss, and enough sample coverage.
    - Add synthetic tests with a rotated soft-iron matrix to prove the new solve handles cross-axis coupling that the current diagonal scale model cannot represent.
