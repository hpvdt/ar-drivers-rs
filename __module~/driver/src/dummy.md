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
- Expose lower and upper bounds for each soft-iron eigenvalue through `DummyConfig` as `soft_iron_min_eigenvalue` and `soft_iron_max_eigenvalue`.
- Compute the permanent vector-norm bounds `sqrt(soft_iron_min_eigenvalue)` and `sqrt(soft_iron_max_eigenvalue)` once during fixture initialization.
- Generate three seeded random vectors and apply Gram-Schmidt orthogonalization without normalizing them. After orthogonalization, clamp each vector's L2 norm to the permanent vector-norm bounds so the scale remains baked into the vector. Resample a vector if it is degenerate or too close to the span of the preceding vectors.
- Form a matrix `V` from the three mutually orthogonal, scaled vectors and construct the soft-iron matrix as `S = V V^T`. The normalized directions of the vectors are the eigenvectors of `S`, and its eigenvalues are their squared L2 norms, so `S` is symmetric positive definite and every eigenvalue remains within the configured bounds without SVD validation.
- Generate `S` once during fixture initialization and reuse it for every magnetometer reading.

## Non-Sensor Behavior

- Persist display mode through `set_display_mode` / `get_display_mode`.
- Keep the existing dummy FOV and display delay.
- Return an identity IMU-to-display transform instead of panicking.
