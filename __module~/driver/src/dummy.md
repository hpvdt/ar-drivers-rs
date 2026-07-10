# Dummy AR Glasses Integration Fixture Consensus

The dummy AR glasses fixture should be a deterministic, public, configurable simulator that remains the fallback used by `any_glasses_or_dummy()`.

## Public API

- Expose the fixture through `pub mod dummy` and re-export `Dummy`, `DummyConfig`, and `DummySnapshot`.
- Provide `Dummy::new()`, `Dummy::with_seed(u64)`, `Dummy::with_config(DummyConfig)`, `Default`, and `snapshot()`.
- Keep default operation deterministic, immediate, and virtual-time based, with the first emitted event being `AccGyro`.

## Sensor Model

- Emit `AccGyro` and `Magnetometer` events strictly alternating from a 9-axis simulated AR glasses stream.
- Keep all emitted sensor vectors in the native `GlassesEvent` RUB body frame.
- Start level with magnetic north as forward, which is negative Z in RUB.
- Keep the glasses rotating on all three angular axes with deterministic random nonnegative body rates, each nonzero and no faster than 2 RPM.
- Track full linear kinematics internally: position, velocity, acceleration, jerk, and attitude.
- Update smooth linear movement by sampling Gaussian jerk, integrating it into acceleration, velocity, and position, and applying damping/clamps to keep the simulation bounded.

## Noise And Distortion

- Gyro emits RUB angular body rates in rad/s with small Gaussian noise.
- Acc emits physical gravity plus linear acceleration in RUB with medium Gaussian noise.
- Mag emits magnetic north in RUB with magnetic dip clamped to +/-30 degrees.
- Inject hard-iron and soft-iron distortion into magnetometer readings. Let the hard-iron offset drift slowly, while keeping the randomly generated soft-iron matrix fixed for the lifetime of the fixture.
- Keep the default hard-iron bias below 50 microtesla while still giving integration tests a realistic calibration challenge.
- Generate a seeded random orthogonal eigenbasis `Q` and three positive eigenvalues `lambda_i`. Apply `S = Q diag(lambda_i) Q^T`, which is symmetric positive definite and maps the ideal magnetic sphere to a well-formed ellipsoid.
- Expose lower and upper bounds for each `lambda_i` through `DummyConfig` as `soft_iron_min_eigenvalue` and `soft_iron_max_eigenvalue`.
- Sample each `lambda_i` directly from its valid range. This requires neither SVD validation nor rejection sampling.
- Generate the soft-iron matrix once during fixture initialization and reuse it for every magnetometer reading. The eigenbasis remains orthogonal and the eigenvalues remain positive, so `S` stays positive definite.

## Non-Sensor Behavior

- Persist display mode through `set_display_mode` / `get_display_mode`.
- Keep the existing dummy FOV and display delay.
- Return an identity IMU-to-display transform instead of panicking.
