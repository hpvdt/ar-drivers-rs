# Fusion TODO

## Compass calibration follow-ups

These tasks should revise the fusion integration layer, not `mag_calibration.rs`.
`MagCalibrator` is maintained separately; keep offset/scale interpretation and
runtime fallback policy in `FusionState`.

### 1. Avoid solving before enough magnetometer samples have been submitted

Current code to revise:

- `src/fusion/lib.rs`, `FusionState::mag_north`
- Specifically the sequence:
  `self.mag.evaluate_sample_vec(raw_mag);`
  followed immediately by `self.mag.perform_calibration();`

Problem:

`FusionState::mag_north` calls `perform_calibration()` on every accepted compass
reading. `MagCalibrator` may return `None` for failed solves, but it does not
expose whether the fixed-size buffer has been filled with real samples, so early
successful solves may still be based partly on default matrix rows.

Expected revision:

Add readiness tracking in `FusionState`, next to the existing `mag:
MagCalibrator<63>` field. Initialize it in `FusionState::new`, update it in
`FusionState::mag_north` after a raw sample passes the `MIN_MAG_NORM` check, and
only call `perform_calibration()` after the readiness policy says calibration is
allowed. If the policy is not ready, `mag_north` should keep returning the raw
normalized compass direction.

Tests to update or add:

- `src/fusion/naive_cf_test.rs`: verify early magnetometer updates still use the
  raw direction and do not require a calibration solve.
- `src/fusion/mag_calibration_test.rs`: keep tests focused on existing
  `MagCalibrator` behavior; do not add readiness state to `MagCalibrator`.

### 2. Make the no-calibration fallback explicit

Current code to revise:

- `src/fusion/lib.rs`, `FusionState::mag_north`
- Specifically the `None => raw_mag` branch after
  `let calibration = self.mag.perform_calibration();`

Problem:

`FusionState::mag_north` has two different fallback meanings: weak compass input
returns `None`, while an unavailable calibration should still use the raw compass
direction when the raw reading is strong enough. That contract is correct for the
current fusion layer, but it is easy to accidentally rewrite calibration failure
as `None` and skip yaw correction unnecessarily.

Expected revision:

Keep the no-cache fallback policy in `FusionState::mag_north`: when
`perform_calibration()` returns `None`, continue with the raw normalized compass
direction as long as `raw_mag.norm() >= MIN_MAG_NORM`. Do not add a last-good
calibration cache unless a measured runtime problem shows that raw fallback is not
acceptable.

Tests to update or add:

- Add a `FusionState::mag_north` or `NaiveCF::update_mag` test that verifies a
  strong magnetometer reading still produces a yaw correction when calibration is
  unavailable.
- Add or keep coverage that a weak raw magnetometer reading returns `None` and
  makes `NaiveCF::update_mag` skip yaw correction.

### 3. Validate calibration output before applying offset and scale

Current code to revise:

- `src/fusion/lib.rs`, `FusionState::mag_north`
- Specifically the `Some((offset, scale))` branch that converts arrays to
  `Vector3<f32>` and applies `(raw_mag - offset).component_div(&scale)`

Problem:

The fusion layer applies any `Some((offset, scale))` result without checking that
the values are suitable for runtime correction. `MagCalibrator::perform_calibration`
currently filters non-finite values, but the fusion layer still owns the policy
for rejecting unusable correction parameters before division.

Expected revision:

Before applying calibration in `FusionState::mag_north`, reject any offset or scale
component that is non-finite, and reject scale components whose absolute value is
too close to zero. Invalid calibration data should not be applied; it should fall
back to the raw normalized compass direction when the raw reading is strong enough.

Tests to update or add:

- Add coverage that invalid scale values are not applied in `FusionState::mag_north`.
- Preserve the existing behavior that a corrected vector below `MIN_MAG_NORM`
  returns `None`, so `NaiveCF::update_mag` skips yaw correction.
