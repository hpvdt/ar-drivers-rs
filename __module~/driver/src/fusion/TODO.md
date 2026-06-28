# Fusion TODO

## Compass calibration follow-ups

These tasks should revise the fusion integration layer, not `mag_calibration.rs`.
`MagCalibrator` is maintained separately; keep offset/scale interpretation and
runtime discard policy in `FusionState`.

### 1. Avoid solving before enough magnetometer samples have been submitted

Current code to revise:

- `src/fusion/lib.rs`, `FusionState::getCalibratedMag`
- Specifically the sequence:
  `self.mag.evaluate_sample_vec(raw_mag);`
  followed immediately by `self.mag.perform_calibration();`

Problem:

`FusionState::getCalibratedMag` calls `perform_calibration()` on every accepted compass
reading. `MagCalibrator` may return `None` for failed solves, but it does not
expose whether the fixed-size buffer has been filled with real samples, so early
successful solves may still be based partly on default matrix rows.

Expected revision:

Add readiness tracking in `FusionState`, next to the existing `mag:
MagCalibrator<63>` field. Initialize it in `FusionState::new`, update it in
`FusionState::getCalibratedMag` after a raw sample passes the `MIN_MAG_NORM`
check, and only call `perform_calibration()` after the readiness policy says
calibration is allowed. If the policy is not ready, `getCalibratedMag` should
return `None` so the raw magnetometer reading is discarded as a heading input.

Tests to update or add:

- `src/fusion/naive_cf_test.rs`: verify early magnetometer updates are discarded
  until calibration readiness allows a solve.
- `src/fusion/mag_calibration_test.rs`: keep tests focused on existing
  `MagCalibrator` behavior; do not add readiness state to `MagCalibrator`.
