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

### 2. Discard readings when calibration is unavailable

Current code to revise:

- `src/fusion/lib.rs`, `FusionState::getCalibratedMag`
- Specifically the `None => raw_mag` branch after
  `let calibration = self.mag.perform_calibration();`

Problem:

`FusionState::getCalibratedMag` currently treats weak compass input and unavailable
calibration differently: weak input returns `None`, while unavailable calibration
falls back to the raw compass direction. That fallback uses uncalibrated data as a
heading input, so it should be removed.

Expected revision:

Keep the no-calibration discard policy in `FusionState::getCalibratedMag`: when
`perform_calibration()` returns `None`, return `None` and skip yaw correction.
Do not add a last-good calibration cache unless a measured runtime problem shows
that discarding unavailable calibration is not acceptable.

Tests to update or add:

- Add a `FusionState::getCalibratedMag` or `NaiveCF::update_mag` test that verifies
  a strong magnetometer reading is discarded when calibration is unavailable.
- Add or keep coverage that a weak raw magnetometer reading returns `None` and
  makes `NaiveCF::update_mag` skip yaw correction.
