# Magnetometer Calibration Benchmark

## Method

Use the deterministic `regression` cases in `tests/mag_calibration_dummy.rs`: four fixed simulator seeds, once with
co-timestamped accelerometer gravity and once without gravity. The integration test uses the production
`MagCalibrator<1023>`, waits five wall-clock seconds after the first successful correction, and then validates for
twenty seconds with an `18 degree` maximum angular error.

Command:

```bash
cargo test --package ar-drivers --no-default-features \
  --test mag_calibration_dummy regression -- --nocapture
```

These are debug test-profile timings, intended only for before/after comparisons on the same machine. Wall-clock time
also includes the simulator's real-time event pacing.

## Direct-solver baseline

- **Implementation commit:** `f40312b` (calibration code unchanged from `50dc7bb`)
- **Date:** 2026-08-01
- **Test result:** 2 passed, 0 failed
- **Complete benchmark duration:** 618.35 seconds

### Four-seed averages

| Metric | With gravity | Without gravity |
|---|---:|---:|
| Average `evaluate_correct` time | 9.902 ms | 5.117 ms |
| Average successful error | 2.403 deg | 2.466 deg |
| Worst successful error | 8.516 deg | 9.138 deg |
| Average post-warm-up error | 2.407 deg | 2.469 deg |
| Worst post-warm-up error | 8.516 deg | 9.138 deg |
| Time until first success | 50.76 s | 53.70 s |
| Samples until first success | 1023 | 1023 |
| Average total run time | 75.84 s | 78.75 s |
| Average samples per run | 1451 | 1520 |

The direct solver publishes on the first cache-full sample in every fixed-seed run. Gravity slightly improves average
and worst aggregate error, while roughly doubling measured calibration computation time because it adds another
full-cache accumulation and `9 x 9` solve.
