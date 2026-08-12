# AGENTS.md - Fusion Module Guide

## Conventions

### Reference Frames

The library uses multiple coordinate frames:

- **RUB (Right-Up-Back):** Android sensor coordinates used by raw device events.
- **FRD (Forward-Right-Down):** Aerospace coordinates used by fusion state and outputs.
- **Custom frames:** Configurable AHRS output frames.

Treat shared sensor-event documentation and the fusion module as the source of truth for frames and units. Keep frame
transformations explicit and centralized, document device-specific deviations, and use the existing linear-algebra
types.

### Acronyms

Every acronym used in this directory's documentation (this guide, `TODO.md`, `MAG_CALIBRATION_BENCHMARK.md`) must
appear in this list. Add a new acronym here in the same change that introduces it; otherwise spell the term out.

- **AHRS:** Attitude and Heading Reference System.
- **FRD:** Forward-Right-Down aerospace coordinate frame.
- **RMS:** Root Mean Square.
- **RUB:** Right-Up-Back Android sensor coordinate frame.
- **SGD:** Stochastic Gradient Descent.
- **SPD:** Symmetric Positive-Definite.

## Magnetometer calibration

`MagCalibrator<N>` retains finite, nonzero FRD magnetometer samples. Each row may also carry a normalized optional
co-timestamped body-frame FRD gravity direction and a device timestamp. Invalid gravity is ignored without rejecting
the magnetometer sample.

Old samples expire according to `max_sample_lifespan_us`, before the incoming magnetometer is validated. An invalid
magnetometer can therefore change retained support, normalization, and live quality through expiry, but it is not
retained and does not run an optimizer update. The first `N` valid samples fill the cache unconditionally. After the
cache is full, a
k-nearest-neighbor diversity heuristic decides whether a new sample replaces a retained row. Every valid current sample
still participates in one online optimizer update even when diversity rejects it. While no calibration has been
published yet, each valid sample also triggers a ramped number of cache-only replay updates that accelerate cold-start
convergence.

The calibrator maintains the raw first moment and second outer-product moment when rows are appended, replaced, or
expired. Cache normalization and corrected centered covariance are derived from these fixed-size statistics without a
row scan. Before nine retained samples, calibration is explicitly pending with confidence zero. After that model
minimum, a finite SPD working candidate must maintain live confidence at least `0.40` for 64 consecutive valid updates
before it can publish from a partially filled cache.

### Production cache size

The cache must outlast one motion pattern, not merely contain enough rows for nine coefficients. With `N = 255`, one
roughly ten-second dummy motion segment covers a near-planar circle and permits worst-case heading errors above
20 degrees. `FusionState` therefore uses `N = 1023`, spanning several motion segments and keeping the direct-solver
baseline near 9 degrees worst case.

### Physical model

Let `b` be hard-iron offset, `D` the symmetric positive-definite soft-iron distortion, `A = D^-1` its correction, and
`m_i` an ideal unit magnetic vector:

```text
x_i = b + D m_i,
||m_i|| = 1,
m_i = A (x_i - b).
```

For current cache mean `mu` and RMS radius `r`, normalize samples as:

```text
u_i = (x_i - mu) / r.
```

The ellipsoid equation is:

```text
u_i^T Q u_i + q^T u_i = 1.
```

The nine online coefficients are:

```text
theta = [Q00, Q11, Q22, Q01, Q02, Q12, q0, q1, q2].
```

Their sample feature vector is:

```text
phi(u) = [ux^2, uy^2, uz^2, 2 ux uy, 2 ux uz, 2 uy uz, ux, uy, uz].
```

### Convex online objective

The radial algebraic residual and regularizer are:

```text
e_r,i = phi_i^T theta - 1,

J_r = 1 / (2 n) sum_i e_r,i^2
    + lambda / 2 ||Q - c I||_F^2,

lambda = 1e-3,
c = 2.
```

The scaled-identity prior counters algebraic ellipsoid inflation under noise and biases unpublished working state away
from indefinite shapes. In coefficient coordinates its weights are:

```text
R = diag(1, 1, 1, 2, 2, 2, 0, 0, 0).
```

#### Gravity surrogate

The optional gravity term uses the ellipsoid normal:

```text
n_i = Q u_i + q / 2.
```

For normalized gravity `g_i`, the projection is linear in `theta`:

```text
s_i = g_i^T n_i = psi(u_i, g_i)^T theta,

psi(u, g) = [
    gx ux,
    gy uy,
    gz uz,
    gx uy + gy ux,
    gx uz + gz ux,
    gy uz + gz uy,
    gx / 2,
    gy / 2,
    gz / 2,
].
```

The optimizer learns a scalar projection `kappa` and minimizes:

```text
e_g,i = psi_i^T theta - kappa,

J_g = gravity_weight / (2 n_g) sum_i (psi_i^T theta - kappa)^2.
```

`J_r + J_g` is convex and quadratic in `(theta, kappa)`; matrix square roots occur only during physical candidate
conversion, not in the optimizer. Gravity is optional, defaults to weight `0.01`, and `gravity_weight(0)` removes this
term.

This term is a physical surrogate rather than exact magnetic dip. The model gives:

```text
Q (u_i - d) = gamma r A m_i,
```

so the surrogate keeps `g_i^T A m_i` approximately constant instead of exact `g_i^T m_i`. It is exact for isotropic
correction and can be biased by anisotropic soft iron. Fixed-seed with-gravity integration results must therefore be
compared with magnetometer-only results and the recorded direct-solver baseline.

Gravity changes the shared `theta`. Physical candidate conversion still uses only `theta`; there is no second
gravity-refined candidate and no relaxed radial-error allowance for gravity-assisted fits.

### Minibatch update

`minibatch_size` defaults to 32 and is clamped to `1..=N.max(1)`. Each update contains:

1. the current valid sample, whether retained or rejected by diversity;
2. random retained rows for the remaining slots, sampled uniformly with replacement.

When the current sample was retained, its row is excluded from random draws so it occurs exactly once. Magnetometer and
gravity data are always sampled together. Sampling uses a private deterministic SplitMix-style `u64` generator.

#### Cold-start cache replay

While no calibration has been published yet, the sample-anchored update is followed by up to `replay_updates`
additional updates (default 4) whose minibatches contain `replay_minibatch_size` observations (default 8) drawn
uniformly with replacement from the retained rows only. The arriving sample is never a required replay member; once
retained, it is an ordinary cache row that replay may draw like any other. The replay count ramps with the retained
fraction, `replay_updates * matrix_filled / N`, because repeatedly fitting a small, low-coverage cache overfits it and
can strand the working shape outside the publishable region. Replay steps share the current learning rate but do not
advance the step counter, so annealing stays tied to the rate of arriving data rather than to compute.
`replay_updates(0)` disables replay.

For minibatch `B` and gravity subset `G`, the analytic gradients are:

```text
theta_prior = [c, c, c, 0, 0, 0, 0, 0, 0],

gradient_theta = 1 / |B| sum_i e_r,i phi_i
               + gravity_weight / |G| sum_i e_g,i psi_i
               + lambda R (theta - theta_prior),

gradient_kappa = -gravity_weight / |G| sum_i e_g,i.
```

Omit gravity terms when `G` is empty, and add regularization once per update rather than once per observation. The
diagonal feature-energy scales are:

```text
scale_theta,j = 1 / |B| sum_i phi_i,j^2
              + gravity_weight / |G| sum_i psi_i,j^2
              + lambda R_jj
              + epsilon,

scale_kappa = gravity_weight + epsilon.
```

The optimizer divides each gradient component by its scale. Its learning rate decays from a private initial value to a
nonzero floor, and its step norm is bounded. A bounded half-step search accepts only finite updates that lower the same
minibatch objective. Working `Q` may temporarily be indefinite; publication still requires SPD. Preventing all
intermediate indefinite states can stall descent at the SPD boundary even when the convex optimum is valid.

### Changing normalization

Append, replacement, and expiry change `mu` and `r`. Persistent coefficients are analytically rebased.

For:

```text
u_old = t + s u_new,
t = (mu_new - mu_old) / r_old,
s = r_new / r_old,
h = 1 - t^T Q_old t - q_old^T t,
```

the equivalent state is:

```text
Q_new = s^2 Q_old / h,
q_new = s (q_old + 2 Q_old t) / h,
kappa_new = s kappa_old / h.
```

The `kappa` transform follows because the normal for the same raw sample scales by `s / h`. If a radius or `h` is
unusable, only unpublished working state resets to `Q = 2 I`, `q = 0`, clears `kappa`, and restarts the optimizer's
learning-rate schedule. A single centered sample has zero radius, so the first informative gradient requires two
distinct samples even though state exists immediately.

### Candidate conversion and live quality

For a working candidate:

```text
d = -0.5 Q^-1 q,
gamma = 1 + d^T Q d,
M = Q / gamma,
b = mu + r d,
A = M^(1/2) / r.
```

The principal symmetric square root uses a `3 x 3` eigendecomposition. A working candidate is valid only when the
coefficients, normalization, offset, and correction are finite, `Q` is positive-definite, `gamma` is positive, and the
correction condition is at most `10`. Invalid candidates have quality zero; raw samples are never substituted for
corrected samples.

For retained raw-sample mean `mu_raw` and second moment `E[x x^T]`, compute:

```text
C_raw = E[x x^T] - mu_raw mu_raw^T,
C_corrected = A C_raw A^T.
```

The hard-iron offset cancels after centering. Directional coverage is a logarithmic ramp from `1` at corrected
covariance condition `1` to `0` at condition `100`. Physical radial fitness uses the running mean square of
`||A (x - b)|| - 1`, evaluated for each valid current sample after its online update with the same working candidate.
Its update weight is `1 / min(sample_count, minibatch_size)`; the statistic resets whenever working optimizer state
resets. Fitness is a linear ramp from `1` at radial RMS `0` to `0` at radial RMS `0.1`. Live confidence is coverage
times fitness, clamped to `[0, 1]`.

Working coefficients and published correction parameters are separate. The hard-iron offset and soft-iron correction
change only after 64 consecutive valid candidates have confidence at least `0.40`, including while the cache is
partial; an invalid or lower-confidence candidate resets that O(1) streak. Before first publication,
`evaluate_correct` returns a non-error `Pending` result and no vector. After publication, a candidate without the
required streak reports its current confidence while leaving the last published correction in use. Fusion callers use
only `Calibrated` vectors for attitude updates. Correcting a reading remains one matrix-vector multiplication followed
by normalization:

```text
m = A (x - b).
```

### Complexity

For minibatch size `B`:

- online fitting is `O(10 B)`;
- cold-start replay adds `O(10 R B_r)` for `R` ramped replay updates of size `B_r`, only until first publication;
- candidate conversion uses fixed `3 x 3` operations;
- normalization and live-quality maintenance use fixed-size raw moments and are `O(1)` in `N`;
- diversity maintenance is expected `O(N)` for a full cache;
- persistent online-optimizer, moment, and quality state is `O(1)` in `N`.

The call remains `O(N)` overall because sample diversity is linear, but live confidence adds no cache scan and there
are no production `9 x 9` normal-matrix accumulations or solves.

### Diversity neighbor cache

Each retained row stores its nearest other rows as a sorted trusted prefix with a small overshoot pad. Append and
replacement update prefixes in amortized `O(k)` per row. Expiry remaps cached indices, and a row is rescanned only when
its trusted prefix falls below `k`. Configurations with `k` above the fixed cache capacity scan rows directly. Distance
selection operates on squared values and takes square roots only for selected neighbors.

### Known adaptation limitation

Online parameters retain historical gradient influence after a row is replaced or expires. Coordinate rebasing changes
units but does not remove that contribution. The backlog tracks explicit replay or forgetting work needed before sample
lifespan can be interpreted as a strict optimizer-history bound.

### Calibration validation

When changing the calibrator, preserve deterministic coverage for cache expiry and readiness, neighbor-cache
invariants, invalid magnetometer and gravity inputs, minibatch clamping, repeatability, full-SPD and asymmetric
distortion, degenerate samples, stable repeated correction, and last-known-good fallback. Run the focused checks first
from `__module~/driver`:

```bash
cargo test --package ar-drivers --no-default-features --lib fusion::mag_calibration_test
cargo test --package ar-drivers --no-default-features --lib fusion::naive_cf_test
cargo test --package ar-drivers --no-default-features --test mag_calibration_dummy regression -- --nocapture
```

Then run the applicable broad Rust checks from the parent guide.

### Benchmark report format

`MAG_CALIBRATION_BENCHMARK.md` is the chronological audit record for deterministic calibration performance. Keep its
top-level title and a leading `## Method` section. The method must identify the benchmark cases and seeds, gravity
modes, production cache/configuration, warm-up and validation intervals, pass threshold, exact command, build profile,
and any timing caveats. Record host or toolchain changes when they could invalidate a timing comparison.

Add one chronological `##` section for each baseline, implementation, or tuning stage. Do not replace older measured
results. Each stage must contain:

- the implementation commit containing the measured calibration code, plus relevant parameter values;
- the run date, test result, and complete measured wall duration;
- a `###` aggregate-results table; and
- a short interpretation comparing accuracy, publication latency, and computation time with the relevant prior stage.

A complete four-seed table has `Metric`, `With gravity`, and `Without gravity` columns and reports:

1. average `evaluate_correct` computation time in milliseconds;
2. average and worst successful angular error in degrees;
3. average and worst post-warm-up angular error in degrees;
4. average time and sample count until first successful publication; and
5. average total run time and sample count.

Average computation time and angular error aggregate their corresponding calls across seeds; worst values are maxima
across all runs; run durations and sample counts are averages across seeds. Keep units in table cells and retain enough
precision to compare with prior stages. Tables are exempt from the 120-character wrapping rule.

If a tuning run intentionally covers only one mode, state that before the table, report the exact changed setting, and
include comparable columns from earlier stages. If the harness or method changes, document the change before presenting
new numbers and do not describe unlike measurements as a direct speedup or regression. Preserve failed or degraded
results when they explain a retained default or later tuning decision.
