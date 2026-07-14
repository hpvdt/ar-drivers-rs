## High severity

- [ ]  Fit the full soft-iron model emitted by the simulator

    - **Summary:** The calibrator fits only an axis-aligned ellipsoid even though the dummy deliberately emits
      cross-axis soft-iron coupling.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** High
    - **Description:** The current design has no `xy`, `xz`, or `yz` terms and returns three component-wise scales:

      ```rust
      mag[3] = -mag[1] * mag[1];  
      mag[4] = -mag[2] * mag[2];  

      let scale = Vector3::new(temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt());
      ```

      By contrast, `sample_soft_iron` constructs a generally rotated symmetric positive-definite matrix:

      ```rust
      let scaled_eigenvectors = Matrix3::from_columns(&vectors);
      scaled_eigenvectors * scaled_eigenvectors.transpose()
      ```

      A diagonal correction cannot undo the resulting cross-axis coupling, so even otherwise good simulated samples do
      not lie on the axis-aligned ellipsoid assumed by `perform_calibration`.
    - **Recommended fix:** Replace the diagonal `(offset, scale)` fit with a hard-iron offset plus an SPD 3x3 correction
      matrix parameterized by a positive-diagonal Cholesky factor and solved with a robust nonlinear fit. (REVIEW: I'd
      like to implement an alternating optimisation algorithm: the first step lock soft-iron Cholesky factor matrix and
      only optimise hard-iron bias, the second step lock the hard-iron bias and only optimise the soft-iron Cholesky
      factor matrix. Do you think it is possible?)

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
      far too early for the current trajectory to provide useful spatial coverage. The SVD condition number tests the
      algebraic design matrix, not whether the measurements adequately constrain a physical three-dimensional model.
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
    - **Recommended fix:** Validate finite positive shape coefficients and bounded fit residuals before any division or
      square root and return a specific non-physical-fit error containing the rejected coefficients. (REVIEW: Not
      needed, SVD already return an error in case of non-positive shape, Cholesky factor won't return non-positive
      shape)
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

- [ ]  Normalize the calibration design automatically

    - **Summary:** The default `pre_scaler` of `1.0` leaves realistic microtesla inputs poorly scaled across the linear,
      squared, and constant design columns.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** Although the API says a value near the magnetic-field magnitude prevents ill-conditioning, the
      default does no normalization and the production construction path uses that default:

      ```rust
      Self {
          matrix: SMatrix::from_element(1.0),
          pre_scaler: 1.,
          // ...
      }
    
      self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
      ```
      With readings on the order of 50 microtesla, coordinate columns are on the order of `10^1`, squared columns are
      on the order of `10^3`, and the constant column is `1`, needlessly worsening the f32 SVD condition and
      thresholding.
    - **Recommended fix:** Derive a finite positive normalization scale from the accepted sample set inside
      `perform_calibration` and undo it only when returning the fitted parameters. (REVIEW: good, given that the input is always earth magnetic field, what's the recommended
      default pre-scaler? This number should only be set once on construction.)
- [ ]  Apply configured pre-scaling consistently and validate it

    - **Summary:** A non-default `pre_scaler` puts buffered samples and new KNN candidates in different units, while
      zero
      or non-finite values can poison the stored matrix.
    - **Affected module:** `src/fusion/mag_calibration.rs`
    - **Severity:** Medium
    - **Description:** Buffered coordinates are divided by `pre_scaler`, but the full-buffer candidate is scored in raw
      units, and the builder accepts every `f32` value:

      ```rust
      pub fn pre_scaler(self, pre_scaler: f32) -> Self {
          Self { pre_scaler, ..self }
      }
    
      self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
    
      let sample_mean_dist = self.mean_distance_from_single(x.transpose());
      ```
      Consequently a feature intended to improve conditioning changes eviction behavior, and `0.0`, `NaN`, or infinity
      can create invalid stored coordinates even though the raw input passed validation.
    - **Recommended fix:** Validate `pre_scaler` once at the construction or configuration boundary, require it to be
      finite and strictly positive, and store only the validated value. Convert each candidate to stored units before
      computing its KNN distance.

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
