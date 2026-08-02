# Magnetometer Calibration Benchmark

## Method

Use the deterministic `regression` cases in `tests/mag_calibration_dummy.rs`, once with co-timestamped accelerometer
gravity and once without gravity. The fixed simulator seeds are `934786981548549007`, `320366629120039532`,
`800448092538851856`, and `14346460742415463748`. The integration test uses the production `MagCalibrator<1023>`, waits
five wall-clock seconds after the first successful correction, and then validates for twenty seconds with an
`18 degree` maximum angular error.

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

## Online minibatch optimizer

- **Implementation commit:** `b24dd0c`
- **Date:** 2026-08-02
- **Initial gravity weight:** `0.1`
- **Test result:** 2 passed, 0 failed
- **Complete benchmark duration:** 631.31 seconds

### Four-seed averages before gravity tuning

| Metric | With gravity | Without gravity |
|---|---:|---:|
| Average `evaluate_correct` time | 4.454 ms | 4.206 ms |
| Average successful error | 2.634 deg | 2.469 deg |
| Worst successful error | 8.958 deg | 9.161 deg |
| Average post-warm-up error | 2.635 deg | 2.478 deg |
| Worst post-warm-up error | 8.958 deg | 9.161 deg |
| Time until first success | 53.82 s | 53.94 s |
| Samples until first success | 1023 | 1023 |
| Average total run time | 78.86 s | 78.97 s |
| Average samples per run | 1503 | 1503 |

The online magnetometer-only fit preserved average post-warm-up accuracy within `0.009 degree` of the direct baseline
and reduced measured computation time by 17.8%. With the old gravity weight of `0.1`, the ellipsoid-normal surrogate
increased average post-warm-up error by `0.228 degree`, confirming the anisotropic-soft-iron bias anticipated in the
design review.

## Tuned gravity surrogate

The default gravity weight was reduced from `0.1` to `0.01`. The magnetometer-only path is unchanged, so only the four
fixed-seed with-gravity cases were rerun after this adjustment.

- **Implementation commit:** `9573867`
- **Date:** 2026-08-02
- **Test result:** 1 passed, 0 failed
- **With-gravity benchmark duration:** 315.90 seconds

### Four-seed tuned result

| Metric | Direct gravity baseline | Online gravity at 0.1 | Online gravity at 0.01 |
|---|---:|---:|---:|
| Average `evaluate_correct` time | 9.902 ms | 4.454 ms | 4.411 ms |
| Average successful error | 2.403 deg | 2.634 deg | 2.484 deg |
| Worst successful error | 8.516 deg | 8.958 deg | 9.139 deg |
| Average post-warm-up error | 2.407 deg | 2.635 deg | 2.493 deg |
| Worst post-warm-up error | 8.516 deg | 8.958 deg | 9.139 deg |
| Samples until first success | 1023 | 1023 | 1023 |

Reducing the surrogate weight recovered most of the average-accuracy regression: tuned post-warm-up error is
`0.086 degree` above the direct gravity baseline and `0.015 degree` above the online magnetometer-only result. The tuned
gravity path is 55.5% faster than the direct gravity implementation. Worst fixed-seed error remains close to the
magnetometer-only online result and well below the integration test's `18 degree` limit.

The result supports keeping the convex gravity surrogate at low default weight. It does not establish physical
equivalence to magnetic dip; broader distortion and adaptation sweeps remain tracked in `TODO.md`.

## Cold-start cache replay

Each valid sample now triggers, in addition to the sample-anchored update, up to four cache-only replay updates of
eight observations while the calibration is unpublished. Replay minibatches are drawn uniformly with replacement from
the retained rows only; the replay count ramps with the retained fraction, and replay steps reuse the current learning
rate without advancing its schedule.

- **Implementation commit:** `b91cf30`
- **Replay updates:** `4` (ramped by `matrix_filled / N`, unpublished phase only)
- **Replay minibatch size:** `8`
- **Date:** 2026-08-02
- **Test result:** 2 passed, 0 failed
- **Complete benchmark duration:** 627.80 seconds

### Four-seed averages

| Metric | With gravity | Without gravity |
|---|---:|---:|
| Average `evaluate_correct` time | 4.414 ms | 4.064 ms |
| Average successful error | 2.499 deg | 2.491 deg |
| Worst successful error | 9.083 deg | 9.106 deg |
| Average post-warm-up error | 2.498 deg | 2.492 deg |
| Worst post-warm-up error | 9.083 deg | 9.106 deg |
| Time until first success | 53.32 s | 53.58 s |
| Samples until first success | 1023 | 1023 |
| Average total run time | 78.34 s | 78.60 s |
| Average samples per run | 1505 | 1504 |

Fixed-seed accuracy and publication latency are unchanged within noise against the tuned-gravity stage: average
post-warm-up error moved by at most `0.014 degree`, worst errors improved by about `0.06 degree` in both modes, and
first success still arrives with the full cache at sample 1023, which floors the latency metric in this harness.
Measured computation time is flat with gravity and 3.4% lower without gravity; the decrease is timing noise rather
than a speedup, because replay only adds pre-publication work and cannot remove any. Replay adds no visible cost
because it is gated to the unpublished phase, where calls skip the `O(N)` publication validation that dominates
post-fill timing.

The convergence benefit the feature targets is not visible in these seeds because the online optimizer was already
near-converged when the cache filled. It is instead covered deterministically by
`mag_calibrator_converges_faster_with_cache_replay`: after one 63-sample fill pass with full-sphere coverage, replay
reaches `0.013` aggregate probe error against `0.376` without, so the working model becomes adequate much earlier when
coverage is sufficient. Two failure modes found during implementation shaped the shipped defaults. Unramped replay
with schedule-advancing steps overfit a small, low-coverage cache (`0.072` vs `0.037` probe error after two asymmetric
passes in the unit scenario); the retained-fraction ramp and the frozen annealing schedule reduced but did not
eliminate that gap on the hostile partial-coverage case (`0.052` vs `0.037`, both converged). A single-observation
replay minibatch never converges, matching the established behavior of a single-observation anchored minibatch.
