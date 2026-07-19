## High severity

- [ ] Avoid recalibrating after every magnetometer reading

    - **Summary:** `evaluate_correct` unconditionally runs the full calibration solve, making the wall-time integration
      result sensitive to machine throughput even when the retained calibration samples have not changed.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** The correction path always invokes `perform_calibration` after evaluating the incoming sample:

      ```rust
      self.evaluate_sample_vec(raw_mag, timestamp_us);
      self.perform_calibration()?;
      ```

      Before timing instrumentation, the default-seed integration test processed 869 magnetometer calls in 20.01
      seconds and failed to complete its five-second post-warmup validation window despite a worst angle of only
      11.967724 degrees. An instrumented rerun processed 944 calls and passed narrowly, with an average
      `evaluate_correct` duration of 21.145257 ms. The randomized run averaged 22.091124 ms over 1,660 calls before an
      unrelated accuracy failure stopped it. A subsequent all-target run failed this same validation-window condition
      for seed `11193037924316477499`: 832 calls averaged 24.008114 ms, while the worst angle remained only 12.395225
      degrees.
    - **Recommended fix:** Have sample evaluation report whether the retained buffer changed, and avoid running all 20
      calibration sweeps when the existing calibration can correct a rejected reading directly. If repeated warm-start
      refinement is still required, schedule it at an explicitly bounded cadence instead of on every sensor reading.

- [ ] Make calibration accuracy robust across simulator seeds

    - **Summary:** The current solver can exceed the 20-degree corrected-heading limit for valid dummy configurations.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** The randomized integration test sampled unique seeds without replacement. Its second round failed
      for seed `5001459942222248181` at virtual timestamp `15070000` microseconds:

      ```text
      corrected magnetometer exceeded 20 degrees at timestamp=15070000: angle_degrees=20.174479
      ```

      This occurred after the unchanged 700-sample warmup and while comparing every corrected reading with the seeded
      simulator's ground truth.
    - **Recommended fix:** Reproduce with the recorded seed and improve the calibration solve or its readiness criteria
      so the existing 20-degree accuracy contract holds without weakening the assertion.

- [ ]  Require redundant samples and three-dimensional coverage before solving

    - **Summary:** Six accepted samples make the six-parameter algebraic system square, but do not make it
      noise-tolerant or prove that the samples cover a three-dimensional ellipsoid.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** Readiness is currently only a row-count check, and the first `N` finite samples are inserted
      unconditionally:

      ```rust
      const DESIGN_MATRIX_COLUMNS: usize = 6;

      if sample_count < DESIGN_MATRIX_COLUMNS {
          return Err(BadMagDataCause::Calibration_InsufficientSamples {
              samples: sample_count,
              required: DESIGN_MATRIX_COLUMNS,
          });
      }

      if self.matrix_filled < N {
          self.add_sample_at(self.matrix_filled, x);
          self.matrix_filled += 1;
      }
      ```

      In the default dummy stream, the sixth magnetometer sample arrives after only about 0.1 seconds of virtual motion,
      far too early for the current trajectory to provide useful spatial coverage.
    - **Recommended fix:** Gate calibration on an overdetermined sample count and a geometric coverage test such as
      per-axis span plus a lower bound on the smallest eigenvalue of the centered sample covariance.
    - **Resolution:** Accepted, set the calibrator to do no correction until the buffer is fully filled
- [ ]  Reject non-ellipsoid solutions before computing scale

    - **Summary:** A fit can pass the design-matrix condition check even when its coefficients do not describe a real
      ellipsoid, making `Calibration_DegenerateScale` a late and imprecise symptom.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** `x[3]`, `x[4]`, or `temp` can be non-positive, but the current code takes square roots and
      divides
      first, then notices only the resulting `NaN` or infinity:

      ```rust
      let offset = Vector3::new(x[0] / 2., x[1] / (2. * x[3]), x[2] / (2. * x[4]));
      let temp = x[5]
          + offset[0] * offset[0]
          + x[3] * offset[1] * offset[1]
          + x[4] * offset[2] * offset[2];
      let scale = Vector3::new(temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt());

      for component in offset.iter().chain(scale.iter()) {
          if !component.is_finite() {
              return Err(BadMagDataCause::Calibration_DegenerateScale { offset, scale });
          }
      }
      ```

      A well-conditioned least-squares system can still yield negative shape coefficients or excessive residual error,
      especially when its input covers only a noisy plane.
    - **Recommended fix:** Replace hard-boundary rejection with soft-boundary regularisation in the least-squares
      objective. Add a penalty term (e.g. log-barrier or Tikhonov-style ridge on the shape coefficients) that
      discourages negative or near-zero eigenvalues of the soft-iron matrix, so the solver naturally favours
      physically valid ellipsoid parameters. Reject only when the regularised residual still exceeds a bound, returning
      the specific coefficients that were rejected.

- [ ]  Stop treating every isolated calibration sample as useful

    - **Summary:** The diversity-only eviction policy preferentially retains isolated sensor outliers, which can drive
      the algebraic fit toward negative shape coefficients.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** Any finite nonzero sample is accepted, and a candidate replaces the least-isolated buffered point
      whenever its KNN score is larger:

      ```rust
      if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
          return;
      }
    
      if low_mean_dist < sample_mean_dist {
          self.add_sample_at(low_index, x);
      }
      ```

      This policy cannot distinguish useful orientation coverage from Gaussian noise tails or magnetic interference, so
      a long-running buffer can become increasingly dominated by extreme points.
    - **Recommended fix:** Use a robust loss for the initial full-buffer fit. Once a provisional fit exists, reject
      samples by a robust radial-residual threshold before applying KNN diversity eviction. Do not impose a fixed raw
      magnitude range while the hard-iron center is unknown; use explicit sensor saturation limits only when available.

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
- [ ]  Add sample age or calibration epochs to the buffer

    - **Summary:** The calibrator fits one static offset to a timeless buffer, so historical samples remain mixed with
      current samples when hard-iron bias changes.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** `MagCalibrator` stores coordinates but no timestamp, age, or calibration epoch:

      ```rust
      pub struct MagCalibrator<const N: usize> {
          matrix: SMatrix<f32, N, 6>,
          matrix_filled: usize,
          mean_distance: f32,
          pre_scaler: f32,
          k: usize,
      }
      ```
      Spatial-diversity eviction can retain old extreme points indefinitely, so measurements generated around different
      ellipsoid centers can be combined into one non-physical fit.
    - **Recommended fix:** Add an explicit reset, bounded-age window, or calibration-epoch policy so stale samples
      cannot remain in the fit solely because they are spatially isolated. Apply spatial-diversity eviction only within
      the currently eligible sample set.
    - **Resolution:** Accepted, add a maximum sample lifespan setting, with default value set to 1 hour.
