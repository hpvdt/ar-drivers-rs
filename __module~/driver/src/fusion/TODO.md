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

  - **Summary:** The convex gravity surrogate is exact for the ellipsoid normal, but only approximates constant
    magnetic dip when the soft-iron correction is anisotropic, so a large gravity weight can bias the fit.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** High
  - **Unit test:** `src/fusion/mag_calibrator_test.rs` (`mag_calibrator_gravity_surrogate_survives_strong_anisotropy`)
- [ ]  Bound stale optimizer influence after sample expiry

  - **Summary:** Online parameters retain historical gradient influence after a row is replaced or expires, and
    nothing removes that contribution, so `max_sample_lifespan_us` no longer strictly bounds the estimator's
    effective history.
  - **Affected module:** `src/fusion/mag_calibrator.rs`
  - **Severity:** High
  - **Unit test:** `src/fusion/mag_calibrator_test.rs` (`mag_calibrator_online_history_outlives_sample_lifespan`)

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
