## High severity

## Medium severity

- [ ]  Score replacement candidates in their post-replacement buffer

    - **Summary:** Candidate scoring should use the same remaining neighbors as the buffered point it might replace.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** The helper skips the smallest distance as a presumed zero self-distance, but the current
      candidate path calls it before inserting the candidate:

      ```rust
      squared_dists.iter().skip(1).take(k)
    
      let sample_mean_dist = self.mean_distance_from_single(x.transpose());
      if low_mean_dist < sample_mean_dist {
          self.add_sample_at(low_index, x);
      }
      ```
      This drops the candidate's actual nearest buffered neighbor. Temporarily replacing the selected victim first gives
      the candidate a real zero self-distance and compares both points against the same remaining `N - 1` samples.
    - **Recommended fix:** Select and save the least-useful buffered row, temporarily replace it with the candidate, and
      score the inserted candidate while skipping its zero self-distance. Commit the replacement only when that score
      exceeds the saved row's score; otherwise restore the complete row. Recompute cached aggregate distance after a
      committed replacement.
- [ ]  Replace full-buffer k-NN rescan with an incremental neighbor cache

    - **Summary:** Each new sample triggers an O(N^2 log N) all-pairs k-NN rescan; caching per-row neighbor sets
      makes the diversity heuristic expected O(N) per sample.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** Once the buffer is full, `evaluate_sample_vec` calls `lowest_mean_distance_by_index`, which
      runs `mean_distance_from_all`: N invocations of `mean_distance_from_single`, each computing N distances and
      fully sorting them:

      ```rust
      squared_dists.sort_unstable_by(|a, b| a.total_cmp(b));

      let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
      ```

      That is N x O(N log N) = O(N^2 log N) per new sample (about 10^7 comparisons at the production N = 1023),
      paid even when the candidate is ultimately rejected. All other per-sample work (expiry compaction, the direct
      ellipsoid fit, and gravity refinement) is O(N) with fixed 9x9 solves, so this heuristic dominates calibrator
      CPU cost.
    - **Recommended fix:** Cache per-row k-NN state (the k smallest distances plus a small overshoot pad) alongside
      each row. On append or replace, compute distances from the new point to all rows once (O(N)), update each
      row's cached set in amortized O(k), rescanning a row only when its pad is exhausted, and rebuild the replaced
      row's own set with `select_nth_unstable_by` (O(N)). During expiry compaction, remap cached neighbor indices
      using the retained-index mapping and mark rows that referenced expired samples dirty. Derive each row's mean
      distance and the buffer-wide argmin from the cache, giving expected O(N) per new sample (worst case O(N^2)
      only under degenerate neighbor churn). While touching this code, defer `sqrt` until after selection and skip
      the self-entry by index instead of `skip(1)`. A k-d tree with reverse-k-NN tracking and an argmin heap would
      reach O(k log N) expected but is not required to get under the O(N log N) bound.
