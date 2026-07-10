distorted + self.sample_noise_vec(seldistorted + self.sample_noise_vec(sel Magnetometer calibration and simulation issues

## Reproduction evidence

With the default deterministic `Dummy` seed, a bounded run of 1,000 magnetometer events through the same
`FusionState::getCalibratedMag` path used by `read_sensors.rs` produced 5 insufficient-sample results, 1
condition-number rejection, 938 `Calibration_DegenerateScale` results, 2 weak calibrated readings, and only 54
successful calibrations. All 54 successes occurred within the first 255 magnetometer events; events 256 through 1,000
all returned `Calibration_DegenerateScale`. The generated fixed angular-rate vector had norm `0.0997171 rad/s`, and the
generated soft-iron matrix contained nonzero off-diagonal terms. The example itself was not changed and is not an
affected module in the findings below.

## High severity

- [ ] Reject non-ellipsoid solutions before computing scale

  - **Summary:** `perform_calibration` can pass the design-matrix condition check even when the fitted coefficients do
    not describe a real ellipsoid. `Calibration_DegenerateScale` is then a late symptom of an invalid conic fit.
  - **Affected module:** `src/fusion/mag_calibration.rs`
  - **Severity:** High
  - **Description:** `x[3]`, `x[4]`, or `temp` can be non-positive. The current code takes square roots first and only
    notices the resulting `NaN` or infinity afterward:

    ```rust
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

    Validate the quadratic before the square roots, return a specific non-ellipsoid-fit error, and reject fits with
    excessive residual error even when the design matrix is well-conditioned. The minimum diagonal-model guard is:

    ```rust
    if !x[3].is_finite()
        || !x[4].is_finite()
        || !temp.is_finite()
        || x[3] <= MIN_POSITIVE_SHAPE
        || x[4] <= MIN_POSITIVE_SHAPE
        || temp <= MIN_POSITIVE_SHAPE
    {
        return Err(/* invalid physical fit, including the coefficients */);
    }
    ```

    This guard should complement, not replace, the SVD rank and condition checks.
- [ ] Require redundant samples and three-dimensional coverage before solving

  - **Summary:** Six samples make the six-parameter algebraic system square, but do not provide a noise-tolerant fit or
    prove that the samples cover a three-dimensional ellipsoid.
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

    In the default dummy stream, the sixth magnetometer sample arrives after only about 0.1 seconds of motion, while
    the first 255 samples span about 5.1 virtual seconds. Add a separate readiness check requiring more samples than
    parameters and a geometric coverage measure, such as per-axis span plus a lower bound on the smallest eigenvalue
    of the centered sample covariance. Report insufficient coverage instead of attempting a fit that later becomes
    `Calibration_DegenerateScale`.
- [ ] Fit a full SPD soft-iron correction instead of a diagonal scale

  - **Summary:** The calibrator cannot represent cross-axis soft-iron coupling because it fits only an axis-aligned
    ellipsoid and returns three component-wise scales.
  - **Affected module:** `src/fusion/mag_calibration.rs`
  - **Severity:** High
  - **Description:** The design contains no `xy`, `xz`, or `yz` terms:

    ```rust
    mag[3] = -mag[1] * mag[1];
    mag[4] = -mag[2] * mag[2];

    let scale = Vector3::new(temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt());
    ```

    Replace `(offset: [f32; 3], scale: [f32; 3])` with
    `(offset: Vector3<f32>, correction: Matrix3<f32>)`. Estimate nine parameters: hard-iron offset `b` plus a
    lower-triangular Cholesky factor `L` with positive diagonal, so `A = L * L.transpose()` is SPD by construction.
    Minimize the unit-sphere residual

    ```text
    ||L.transpose() * (sample - b)||^2 - 1
    ```

    using Levenberg-Marquardt and an analytic Jacobian. Initialize `b` from the sample midpoint or mean, initialize
    `L` diagonally from per-axis extents, and initialize off-diagonal terms to zero. Apply calibration as
    `correction * (raw_mag - offset)`, then normalize it for heading use. Validate finite parameters, positive
    Cholesky diagonal, condition number, residual loss, and coverage. Preserve sample validation, buffering, KNN
    scoring, and eviction behavior, and add a deterministic rotated-soft-iron test.s.
- [ ] Generate a non-planar attitude trajectory

  - **Summary:** Three nonzero components in one constant angular-rate vector still produce rotation about one fixed
    axis, so the ideal magnetometer trace is a planar circle rather than three-dimensional coverage.
  - **Affected module:** `src/sim/dummy.rs`
  - **Severity:** High
  - **Description:** The dummy samples one angular-rate vector during construction and reuses it forever:

    ```rust
    let angular_rate_rub = sample_angular_rate(&mut rng, config.max_body_rate_rpm);

    let rotation_increment = UnitQuaternion::from_scaled_axis(self.angular_rate_rub * dt);
    self.attitude *= rotation_increment;
    ```

    A fixed affine magnetometer distortion maps that circle to another planar ellipse. Without noise or hard-iron
    drift, all samples obey one plane equation and the calibration design is rank-deficient. Noise can make the
    matrix appear full-rank without supplying real orientation coverage. Change the body-rate direction over time
    using a smooth deterministic maneuver or seeded multi-axis excitation schedule that visits substantially
    different roll, pitch, and yaw orientations.
- [ ] Stop treating every isolated calibration sample as useful

  - **Summary:** The buffer has no plausibility or robust-residual gate, so its diversity-only eviction policy
    preferentially retains sensor outliers.
  - **Affected module:** `src/fusion/mag_calibration.rs`
  - **Severity:** High
  - **Description:** Any finite, nonzero sample is accepted, and an isolated candidate replaces the least-isolated
    buffered point:

    ```rust
    if !x.iter().all(|e| e.is_finite()) || x.norm_squared() <= f32::EPSILON {
        return;
    }

    if low_mean_dist < sample_mean_dist {
        self.add_sample_at(low_index, x);
    }
    ```

    Add robust sample admission before KNN eviction, such as a sensor-range check plus a robust residual or
    neighborhood-consistency gate. Use a robust fit loss so a small number of retained outliers cannot force negative
    shape coefficients. The calibrator must remain safe for finite outliers regardless of which device or simulator
    produced them.

## Medium severity

- [ ] Balance default angular motion against magnetometer noise

  - **Summary:** The default dummy changes the noiseless magnetic vector much more slowly than it perturbs each
    measurement, so early spatial diversity is dominated by noise rather than orientation.
  - **Affected module:** `src/sim/dummy.rs`
  - **Severity:** Medium
  - **Description:** Defaults combine a maximum of 1 RPM per body axis, 20 ms between magnetometer samples, a 50
    microtesla field, and 1.5 microtesla per-component noise:

    ```rust
    event_period_us: 10_000,
    max_body_rate_rpm: 1.0,
    mag_noise_std_dev: 1.5,
    magnetic_field_strength: 50.0,
    ```
    Even at the maximum combined rate, the ideal field changes by only about 0.18 microtesla between magnetometer
    samples, while the standard deviation of the difference between two independent noisy samples is about 2.1
    microtesla per component. For the measured default seed, the ideal change is limited to about 0.10 microtesla per
    sample. Add a calibration-oriented motion profile, reduce noise for that profile, or make slow/noisy behavior an
    explicitly selected stress profile.
- [ ] Include the true nearest neighbor when scoring a new sample

  - **Summary:** `mean_distance_from_single` always skips the smallest distance as a presumed zero self-distance, but
    a new candidate is not yet in the buffer and has no self-distance.
  - **Affected module:** `src/fusion/mag_calibration.rs`
  - **Severity:** Medium
  - **Description:** The same helper is used for buffered rows and new candidates:

    ```rust
    squared_dists.iter().skip(1).take(k)

    // Buffered row: one distance is the row's zero self-distance.
    mean_dist[i] = self.mean_distance_from_single(row.into());

    // New sample: every distance is to a real buffered neighbor.
    let sample_mean_dist = self.mean_distance_from_single(x.transpose());
    ```
    For a candidate, `skip(1)` discards its actual nearest neighbor and exaggerates its isolation. Give the helper an
    explicit `skip_self` argument or use separate helpers, so buffered rows skip the known zero distance and new
    candidates average their true nearest `k` distances.
- [ ] Model finite magnetometer range instead of unbounded noise output

  - **Summary:** The simulator draws magnetometer noise from an unbounded Gaussian and never applies sensor saturation,
    so a long-running immediate stream eventually emits arbitrarily extreme but finite readings.
  - **Affected module:** `src/sim/dummy.rs`
  - **Severity:** Medium
  - **Description:** Noise samples are added directly to the distorted field:

    ```rust
    distorted + self.sample_noise_vec(self.config.mag_noise_std_dev)

    Normal::new(0.0, std_dev as f64)
        .map(|normal| normal.sample(rng) as f32)
        .unwrap_or(0.0)
    ```
    Add a configurable magnetometer measurement range and saturate or reject simulated readings outside that range.
    This makes the dummy match a finite-range sensor and prevents rare Gaussian tails from becoming unlimited
    calibration inputs. This simulator fix is separate from the calibrator's obligation to reject outliers safely.
- [ ] Add sample age or forgetting to the calibration buffer

  - **Summary:** The calibrator fits one offset to a timeless buffer, so historical samples remain mixed with current
    samples when the magnetic environment or hard-iron bias changes.
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
    Add an explicit forgetting or reset policy and test the maximum trackable offset-change rate. Eviction must
    consider sample age or calibration epoch in addition to spatial diversity; otherwise samples from different
    ellipsoid centers can remain in one static fit indefinitely.
- [ ] Separate static-calibration and drifting-bias simulator profiles

  - **Summary:** The default simulator continuously changes its hard-iron offset, so one deterministic run does not
    provide a stationary ellipsoid for validating a static calibration solve.
  - **Affected module:** `src/sim/dummy.rs`
  - **Severity:** Medium
  - **Description:** The default has a five-minute drift cycle:

    ```rust
    hard_iron_drift: Vector3::new(2.0, 1.5, 2.5),
    hard_iron_drift_period_us: 5 * 60 * 1_000_000,

    self.config.hard_iron_base
        + self.config.hard_iron_drift.component_mul(&drift_shape)
    ```
    Provide a static calibration-validation profile with `hard_iron_drift: Vector3::zeros()` and retain drifting
    bias as a separate adaptive-calibration stress profile. This keeps both behaviors testable without making the
    baseline calibration fixture change its target parameters during the solve.
