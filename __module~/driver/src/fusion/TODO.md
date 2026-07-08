# Fusion TODO

## Compass calibration follow-ups

These tasks should keep the runtime discard policy in `FusionState`, but may
revise `MagCalibrator` when the validity check depends on the calibration solve
itself.

### 1. Reject ill-conditioned magnetometer calibration solves

Current code to revise:

- `src/fusion/mag_calibration.rs`, `MagCalibrator::perform_calibration`
- Specifically the sequence:
  `(self.matrix.transpose() * self.matrix).try_inverse()`
  followed by applying that inverse to the least-squares right-hand side.
- `src/fusion/lib.rs`, `FusionState::getCalibratedMag`
- Specifically its handling of `self.mag.perform_calibration()?`

Problem:

The current rejection criterion is only whether the normal-equation matrix can
be inverted with `try_inverse()`. That rejects singular sample sets, but it can
still accept extremely ill-conditioned solves. A small fixed-size buffer can be
filled quickly with samples concentrated in one direction or one small region;
such data may produce an invertible matrix while still giving a poorly
constrained offset/scale estimate.

Expected revision:

Before accepting a calibration result, compute a numerical conditioning measure
for the same normal-equation matrix used by the current solver. Because the
implementation currently uses `try_inverse()` and not SVD, start with a
condition proxy such as `normal.norm() * inverse.norm()` after
`try_inverse()` succeeds, with an empirically chosen maximum condition number.
If the matrix is singular or exceeds that conditioning threshold,
`perform_calibration()` should return a `BadMagDataCause` and
`FusionState::getCalibratedMag` should continue discarding the raw
magnetometer reading as heading input.

Do not replace this with a simple "number of accepted samples" gate. A full
buffer is not enough evidence that the calibration problem is observable; the
accepted samples must also span enough independent directions to produce a
stable solve. If the solver is later changed to QR or SVD, move the criterion to
the decomposition's rank/condition information instead of keeping the
normal-equation proxy.

Tests to update or add:

- `src/fusion/mag_calibration_test.rs`: add concentrated or nearly collinear
  sample sets that are not accepted as valid calibration solves.
- `src/fusion/naive_cf_test.rs`: verify that an ill-conditioned calibration
  result is discarded and does not update heading correction.
