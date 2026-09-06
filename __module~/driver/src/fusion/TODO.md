## High severity

- [ ]  Correct XREAL Air 1 magnetometer deserialization against replay calibration

  - **Summary:** The committed Air 1 packet trace produces calibrated readings, but the gravity-assisted replay
    regression exposes magnetometer data that is inconsistent with the accelerometer frame.
  - **Affected modules:** `src/xreal_air.rs`, `tests/xreal_air_replay.rs`
  - **Severity:** High
  - **Description:** Over the first 60-second trace cycle, calibration without gravity publishes after `4.658 s` and
    averages `0.625` radial fitness. Calibration with gravity publishes after `10.972 s` and averages `0.610` radial
    fitness but only `0.089` gravity fitness, failing the required strict `0.5` floor. Linear acceleration in the
    capture is negligible compared with gravity, and the magnetometer and acceleration observations are separated
    by only a few milliseconds, so neither effect explains the cross-sensor inconsistency. Magnetometer packet
    deserialization or source-to-RUB frame reconstruction remains incorrect.
  - **Recommended fix:** Re-derive the magnetometer byte offsets, field widths, signedness, endianness, scaling, and
    source-axis mapping from the immutable packet fixture and independent implementations. Change only the raw
    magnetometer decoder until both replay modes publish within 60 seconds and average radial and gravity fitness
    strictly above `0.5`; do not weaken those thresholds or modify `MagCalibrator`, fusion, accelerometer, or
    gyroscope behavior.
- [ ]  Validate the ellipsoid-normal gravity surrogate under anisotropic soft iron

  - **Summary:** The planned convex gravity residual is exact for the ellipsoid normal, but only approximates
    constant physical magnetic dip when the soft-iron correction is anisotropic.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** High
  - **Description:** The online objective uses the linear normal projection
    `g_i^T (Q u_i + q / 2)`. From the physical model,

    ```text
    Q (u_i - d) = gamma r A m_i,
    ```

    so the surrogate keeps `g_i^T A m_i` approximately constant rather than the physical dip `g_i^T m_i`.
    These coincide for isotropic correction but can differ for the full rotated SPD distortion emitted by the
    simulator. A gravity weight that is too large can therefore bias the shape toward isotropy even though the
    combined optimization problem is convex and quadratic. The fixed-seed benchmark found this bias at weight `0.1`;
    reducing the default to `0.01` recovered average post-warm-up accuracy to within `0.086 degree` of the direct
    gravity baseline.
  - **Recommended fix:** Extend validation beyond the current fixed simulator distortion with stronger anisotropy,
    rotated eigenvectors, inconsistent acceleration, and multiple magnetic dip angles. Keep the low default weight or
    disable the surrogate if those sweeps show a repeatable regression.
- [ ]  Bound stale optimizer influence after sample expiry

  - **Summary:** Persistent SGD parameters remember gradients from rows that no longer satisfy the cache lifespan.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** High
  - **Description:** The direct solver is exactly a function of the current retained rows. An online optimizer keeps
    historical parameter updates after a row is replaced or expires, so `max_sample_lifespan_us` no longer strictly
    bounds the estimator's effective history. Coordinate rebasing preserves the represented ellipsoid but does not
    remove an expired row's old gradient contribution.
  - **Recommended fix:** Reset and replay a bounded number of minibatches after expiry, or introduce an explicit
    forgetting schedule whose horizon is no longer than the configured lifespan. Add an adaptive hard-iron drift
    integration case before claiming equivalent expiry semantics.
- [ ]  Remove continuous cache reset so fitness statistics stop resetting

  - **Summary:** Cached samples are continuously renormalized to the drifting cache mean and radius; unusable
    rebases wipe the working optimizer state and the running RMS fitness statistics, producing block-long dips to
    zero.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** High
  - **Description:** Every append, replacement, and expiry shifts `mu` and `r`, so each retained row is renormalized
    as `u = (x - mu) / r` and the persistent coefficients are analytically rebased. When a radius or the rebase
    scalar `h = 1 - t^T Q t - q^T t` is unusable, unpublished working state resets to `Q = 2 I`, which also resets
    the radial and gravity RMS statistics. The Air 1 replay then reports fitness ramping from zero back up over a
    block of evaluations several times per cycle even though the decoded data is stable. These dips are a
    normalization-lifecycle artifact, not a data or converg'ence problem, and they force downstream verification to
    tolerate long streaks of near-zero post-warm-up fitness.
  - **Recommended fix:**
    - Each raw magnetometer samples (floating point) in the cache is stored as once and for all.
    - Delete reset/rebase related code, e.g. reset_row_cache and rebuild_row_cache.
    - The continuous tracking of cache mean and radius should be kept, but won't trigger reset
    - everything else (Online SGD, coverage maximizing, fitness/confidence estimation) should be preserved.
    - The improved version should not make the code any longer (excluding comments). With useless state removed
  - **Verification:**
    - run the regression benchmark before the change to record this host's baseline
      (`cargo test --package ar-drivers --no-default-features --test mag_calibrator_sim_motion regression -- --nocapture`).
    - Acceptance: the Air 1 replay
      report no longer shows block-long post-warm-up dips to zero.
      Record a new chronological stage in `MAG_CALIBRATION_BENCHMARK.md` with before and
      after tables.
    - Scope: do not bundle the stale-optimizer-influence item; rebasing will no longer exist, so
      adjust that item's wording separately.
    -

## Medium severity

- [ ]  Score replacement candidates in their post-replacement buffer

  - **Summary:** Candidate and victim diversity scores currently use different neighbor pools.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** Medium
  - **Description:** The victim's score excludes itself, while the candidate is scored against all `N` old rows,
    including the row it would replace:

    ```rust
    let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
    let squared_dists = self.squared_distances_to(x, N);
    let mut scratch = squared_dists;
    let sample_mean_dist = Self::mean_of_smallest(&mut scratch, k);
    ```
    A candidate close to the selected victim can be rejected because that soon-to-be-evicted row contributes to its
    nearest-neighbor score. This also changes which transient observations receive persistent cache representation,
    although every valid current observation still receives one online gradient update.
  - **Recommended fix:** After selecting the victim, exclude `low_index` from the candidate's distance scratch before
    selecting its `k` nearest neighbors. Compare both scores against the same `N - 1` retained rows, then update the
    incremental neighbor cache only after accepting the replacement.
