## High severity

- [ ] Validate the ellipsoid-normal gravity surrogate under anisotropic soft iron

    - **Summary:** The planned convex gravity residual is exact for the ellipsoid normal, but only approximates
      constant physical magnetic dip when the soft-iron correction is anisotropic.
    - **Affected module:** `src/fusion/mag_calibration.rs`
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

- [ ] Bound stale optimizer influence after sample expiry

    - **Summary:** Persistent SGD parameters remember gradients from rows that no longer satisfy the cache lifespan.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** The direct solver is exactly a function of the current retained rows. An online optimizer keeps
      historical parameter updates after a row is replaced or expires, so `max_sample_lifespan_us` no longer strictly
      bounds the estimator's effective history. Coordinate rebasing preserves the represented ellipsoid but does not
      remove an expired row's old gradient contribution.
    - **Recommended fix:** Reset and replay a bounded number of minibatches after expiry, or introduce an explicit
      forgetting schedule whose horizon is no longer than the configured lifespan. Add an adaptive hard-iron drift
      integration case before claiming equivalent expiry semantics.

## Medium severity

- [x] Replace direction-grid coverage with a directional design-matrix E-optimality score

    - **Resolution:** Implemented with `sqrt(2)` cross-term feature weights (exact rotation invariance; the `2 dx dy`
      convention quoted below is only covariant up to the feature metric). The design sum is recomputed from the
      current cache on each quality update: insertion-time direction snapshots go stale with the drifting cache mean
      (~60x underestimation of the smallest eigenvalue), and centering by the fitted offset destabilizes near-planar
      supports (the offset's thin-axis component is itself unconstrained there). Thresholds re-tuned to `0.03`/`0.02`
      against the nine-seed suite; see the E-optimality stage in `MAG_CALIBRATION_BENCHMARK.md`.
    - **Summary:** Coverage currently counts occupied cells of a fixed `8 x 16` latitude/longitude grid whose cells
      have unequal solid angle and whose frame is not rotation-invariant; replace it with the minimum eigenvalue of
      an incrementally maintained directional feature design matrix.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** The grid bins by `asin(z)`, so polar cells are about five times smaller in solid angle than
      equatorial cells, and the grid is anchored to the body frame, so the score depends on the device's incidental
      orientation. Both defects are quadrature artifacts of approximating a supremum over the sphere with a fixed
      partition. Instead, score coverage with the E-optimality criterion of the algebraic fit restricted to unit
      directions. For each retained row keep the mean-centered unit direction `d_i = (x_i - mu) / ||x_i - mu||` and
      the quadratic feature vector

      ```text
      phi(d) = [dx^2, dy^2, dz^2, 2 dx dy, 2 dx dz, 2 dy dz, dx, dy, dz].
      ```

      Maintain the `9 x 9` design matrix `M = sum_i phi(d_i) phi(d_i)^T` incrementally: add the rank-1 outer product
      when a row is appended or replaces a victim, subtract it on expiry. Coverage equals
      `clamp(lambda_min(M) / lambda_ref, 0, 1)` with `lambda_ref = 2/15`, the smallest design eigenvalue under the
      uniform distribution on the sphere. This is rotation-invariant (eigenvalues rotate covariantly), needs no
      partition, and the least-squares leverage bound
      `Var(phi(u)^T theta) = sigma^2 phi(u)^T M^-1 phi(u) <= sigma^2 ||phi(u)||^2 / lambda_min(M)` certifies that
      the nine ellipsoid coefficients are constrained in every direction. Rank deficiency detects lower-dimensional
      support by construction: a tilted circle leaves rank 5 (columns `dz` and `dz^2`, and `dx` and `dx dz`, are
      proportional), and two perpendicular rings kill exactly the `dy dz` column, so a near-planar pancake cannot
      inflate the score through the fitted correction the way the corrected covariance did.
    - **Recommended fix:** Add per-row stored directions (3 `f32` each) and the `9 x 9` design sum; hook the rank-1
      add/remove into the existing `add_raw_moment`/`remove_raw_moment` sites so maintenance stays `O(1)` per update
      with no cache scan. Recompute `lambda_min` only when the cache mutates, warm-started from the previous
      eigenvalue pair (the design changes by one rank-2 update), keeping each mutation at roughly 1-3k flops; the
      no-mutation path reads one cached scalar. Delete the grid state (`sample_coverage_bins`,
      `coverage_bin_counts`, `coverage_occupied_bins`) and the `asin`/`atan2` binning. Keep
      `confidence = coverage x fitness` and the publication streak. Re-tune `MIN_PUBLICATION_CONFIDENCE`,
      `PUBLICATION_STREAK_RESET_CONFIDENCE`, and `MIN_PUBLICATION_STREAK` against the nine-seed regression suite,
      add tests for rotation invariance, planar rank-deficiency, and identity-versus-distorted equivalence, update
      the coverage description in `AGENTS.md`, and record a new stage in `MAG_CALIBRATION_BENCHMARK.md`.

- [ ] Score replacement candidates in their post-replacement buffer

    - **Summary:** Candidate and victim diversity scores currently use different neighbor pools.
    - **Affected module:** `src/fusion/mag_calibration.rs`
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

- [ ] Avoid full-cache publication validation after unchanged optimizer state

    - **Summary:** Full covariance and radial validation can remain `O(N)` even when an online step is rejected.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** The online gradient is bounded by `minibatch_size`, but deriving and validating a publication
      candidate still scans all retained rows. Revalidating after an invalid input or rejected optimizer step provides
      no new calibration information and can hide the fitting-cost reduction in end-to-end timing.
    - **Recommended fix:** Track a working-parameter revision and rerun full publication validation only after that
      revision or the cache changes. When expiry removes enough support to invalidate the working candidate, update the
      live pending/quality state without restoring a full-buffer `InsufficientSamples` gate.

- [x] Replace full-buffer readiness with a live calibration quality score

    - **Summary:** Publish a bounded `[0, 1]` calibration quality score continuously and use it, rather than a full
      sample cache, to decide when the first valid correction is ready.
    - **Affected module:** `src/fusion/mag_calibration.rs`, its public result/error types, fusion callers, and tests
    - **Severity:** Medium
    - **Description:** The score must add no cache-size-dependent work to an online update. It combines directional
      coverage and radial fitness for the same candidate correction. Coverage is the condition number of the centered,
      unnormalized corrected vectors `A (x_i - b)`, computed as `A C_raw A^T`; maintain the raw first and second
      moments as rows are appended, replaced, or expired so this remains `O(1)` in `N`. The hard-iron offset cancels
      after centering. Fitness is the running mean square of each valid current sample's physical radial residual
      `||A (x - b)|| - 1`, evaluated after its online update with the same working candidate. Update it with
      `alpha = 1 / min(sample_count, minibatch_size)`, reset it whenever the working optimizer state resets, and do not
      rescan the partial or full cache. A candidate that cannot produce finite SPD correction parameters has score
      zero. Never substitute raw samples for corrected samples when computing either physical component.
    - **Recommended fix:** Define coverage as a logarithmic ramp from condition `1 -> 1` to `100 -> 0`, fitness as a
      linear ramp from radial RMS `0 -> 1` to `0.1 -> 0`. Confidence is the product of those two components clamped to
      `[0, 1]`. Before nine accepted samples, or while no finite SPD working candidate exists, report a non-error
      pending state with confidence zero and no corrected vector; do not return raw magnetometer data. A valid working
      candidate must maintain confidence at least `0.40` for 110 consecutive valid updates before publishing the first
      correction, even when the cache is only partially filled. Reset this O(1) streak on an invalid or
      lower-confidence candidate so a transient score spike is not treated as readiness. After publication, a
      candidate without the required streak leaves the last published correction available and reports the current
      confidence.
      Surface both pending and calibrated states through `evaluate_correct`, add a `get_confidence()` getter, remove
      `InsufficientSamples` as the cache-readiness result, and update fusion callers so only calibrated vectors enter
      attitude estimation. Add deterministic tests for partial-cache publication, pending-state handling, corrected
      covariance under anisotropic distortion, zero quality for invalid candidates, last-published fallback, and the
      absence of a per-update full-cache confidence scan.
