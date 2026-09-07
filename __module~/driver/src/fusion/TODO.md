## High severity

- [x]  Correct XREAL Air 1 magnetometer deserialization against replay calibration

  - **Summary:** The magnetometer axis mapping in the version-2 report was corrected to
    `(scaled[1], -scaled[2], -scaled[0])`, so both gravity and non-gravity replay modes now publish within the timeout
    and average radial and gravity fitness strictly above `0.5`.
  - **Affected module:** `src/xreal_air.rs`
  - **Severity:** High
  - **Unit test:** `tests/xreal_air_replay.rs` (`air1_trace_calibrates_with_gravity`,
    `air1_trace_calibrates_without_gravity`)
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
    bounds the estimator's effective history. Working state persists across normalization changes, so an expired
    row's old gradient contribution is never removed.
  - **Recommended fix:** Reset and replay a bounded number of minibatches after expiry, or introduce an explicit
    forgetting schedule whose horizon is no longer than the configured lifespan. Add an adaptive hard-iron drift
    integration case before claiming equivalent expiry semantics.

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
