# Online Magnetometer Calibration Plan

## Summary

Replace the full-buffer `9 x 9` direct ellipsoid solve and the subsequent gravity-refinement solve with one online,
convex-quadratic minibatch objective. Preserve the current sample reservoir, physical candidate conversion, calibration
quality gates, cache-full readiness rule, and last-known-good correction.

The implementation will:

- optimize the existing nine ellipsoid coefficients online;
- add a convex gravity surrogate based on the ellipsoid normal;
- always include the current valid sample, even when the diversity cache rejects it;
- draw the remaining observations randomly from the retained cache;
- expose the maximum work per update through `minibatch_size`;
- publish a correction only after the cache is full and the candidate passes the existing quality checks.

## Goals

- Begin maintaining optimizer state as soon as useful samples arrive.
- Remove both production `9 x 9` normal-system solves.
- Keep the online update bounded by configurable minibatch size.
- Keep the total training objective convex and quadratic.
- Preserve full symmetric positive-definite hard- and soft-iron calibration.
- Preserve deterministic behavior for identical input streams.
- Match the direct solver's fixed-seed accuracy while reducing fitting cost.

## Non-goals

- Changing the k-nearest-neighbor diversity policy.
- Reducing the production cache from `N = 1023`.
- Returning provisional calibrated readings before the cache is full.
- Making gravity mandatory.
- Adding asynchronous optimization or a machine-learning dependency.
- Claiming that the gravity surrogate is identical to physical magnetic dip under arbitrary anisotropic soft iron.

## Existing behavior to preserve

- Cache only finite, nonzero FRD magnetometer samples.
- Normalize finite, nonzero co-timestamped FRD gravity directions before use.
- Expire old rows before validating the incoming sample.
- Fill the first `N` rows unconditionally, then use the existing diversity replacement rule.
- Require `N.max(9)` retained rows, which means a full cache and `N >= 9` in practice.
- Reject poorly covered samples, non-SPD candidates, ill-conditioned corrections, and excessive radial RMS.
- Keep published hard- and soft-iron parameters unchanged after a rejected update.
- Return `InsufficientSamples` after expiry drops the cache below readiness, even when an older correction exists.
- Normalize successful corrected readings before returning them.

## Parameterization

For retained raw sample `x_i`, cache mean `mu`, and RMS radius `r`, define:

```text
u_i = (x_i - mu) / r.
```

The normalized ellipsoid is:

```text
u_i^T Q u_i + q^T u_i = 1,
```

with parameter vector:

```text
theta = [Q00, Q11, Q22, Q01, Q02, Q12, q0, q1, q2].
```

Its linear feature vector is:

```text
phi(u) = [ux^2, uy^2, uz^2, 2 ux uy, 2 ux uz, 2 uy uz, ux, uy, uz].
```

The optimizer also maintains one scalar `kappa`, the learned constant projection of gravity onto the ellipsoid normal.
The complete optimizer state therefore contains ten scalar parameters, while physical candidate extraction continues to
use only `theta`.

## Joint convex-quadratic objective

### Radial algebraic term

Retain the current regularized algebraic objective:

```text
e_r,i = phi_i^T theta - 1,

J_r = 1 / (2 n) sum_i e_r,i^2
    + lambda / 2 ||Q - c I||_F^2,

lambda = 1e-3,
c = 2.
```

The regularizer uses:

```text
R = diag(1, 1, 1, 2, 2, 2, 0, 0, 0).
```

### Gravity surrogate

The ellipsoid normal, apart from an irrelevant factor of two, is:

```text
n_i = Q u_i + q / 2.
```

For normalized gravity `g_i`, its projection is linear in `theta`:

```text
s_i = g_i^T n_i = psi(u_i, g_i)^T theta,
```

where:

```text
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

Use the learned constant `kappa` to avoid a whole-cache mean during online updates:

```text
e_g,i = psi_i^T theta - kappa,

J_g = gravity_weight / (2 n_g) sum_i e_g,i^2.
```

The total objective `J_r + J_g` is convex and quadratic in `(theta, kappa)`. Its Hessian is positive semidefinite; the
regularizer and sufficiently diverse data make the fitted system usable. No matrix square root occurs in the optimizer.

This gravity term is a deliberate surrogate. From the physical model:

```text
Q (u_i - d) = gamma r A m_i,
```

so it keeps `g_i^T A m_i` approximately constant rather than the exact dip `g_i^T m_i`. The distinction vanishes for
isotropic correction and grows with anisotropic soft iron. Fixed-seed tests with full SPD distortion are therefore the
acceptance criterion for retaining this term.

## Analytic minibatch gradient

For minibatch `B` and its gravity-tagged subset `G`:

```text
gradient_theta = 1 / |B| sum_i e_r,i phi_i
               + gravity_weight / |G| sum_i e_g,i psi_i
               + lambda R (theta - c e_d),

gradient_kappa = -gravity_weight / |G| sum_i e_g,i.
```

Omit the gravity contribution when `G` is empty. Add regularization once per update, not once per sample.

Use diagonal feature-energy normalization to avoid unit-dependent learning rates:

```text
scale_theta,j = 1 / |B| sum_i phi_i,j^2
              + gravity_weight / |G| sum_i psi_i,j^2
              + lambda R_jj
              + epsilon,

scale_kappa = gravity_weight + epsilon.
```

Apply a bounded normalized SGD step. A short half-step search may reject non-finite, non-SPD, or minibatch-loss-
increasing candidates. The step size, bound, and retry count remain private constants tuned against deterministic tests.

## Minibatch construction

- `minibatch_size` includes the mandatory current valid observation.
- Include the current observation even if the diversity cache rejects it.
- If the current observation was retained, exclude that row from random draws so it occurs exactly once.
- Draw remaining rows uniformly with replacement from eligible retained rows.
- Draw magnetometer and optional gravity data together.
- Invalid magnetometer input performs no optimizer step, although it may already have triggered expiry.
- Use a private deterministic `u64` generator so identical streams select identical rows.

Add the consuming builder:

```rust
/// Configure the maximum observations used by each online optimizer update.
/// The current valid observation is always included. The default is 32.
pub fn minibatch_size(self, minibatch_size: usize) -> Self
```

Clamp the value to `1..=N.max(1)`. Do not initially expose learning rate or RNG seed.

## Normalization changes

The cache mean and radius change during append, replacement, and expiry. Working parameters must be rebased rather than
silently interpreted in different coordinates.

For old and new normalized coordinates:

```text
u_old = t + s u_new,
t = (mu_new - mu_old) / r_old,
s = r_new / r_old.
```

Define:

```text
h = 1 - t^T Q_old t - q_old^T t.
```

The equivalent new parameters are:

```text
Q_new = s^2 Q_old / h,
q_new = s (q_old + 2 Q_old t) / h,
kappa_new = s kappa_old / h.
```

The `kappa` transform follows because the new ellipsoid normal equals `s / h` times the old normal for the same raw
sample. If either radius or `h` is unusable, reset unpublished working state to `Q = 2 I`, `q = 0`, and uninitialized
`kappa`.

A single centered sample has zero radius. It initializes optimizer state but cannot produce an informative gradient;
the first informative update occurs after distinct samples establish nonzero radius.

## Candidate extraction and publication

The online optimizer replaces only coefficient estimation. Keep the current physical conversion:

```text
d = -0.5 Q^-1 q,
gamma = 1 + d^T Q d,
A = (Q / gamma)^(1/2) / r,
b = mu + r d.
```

Before publication, require:

- a full cache and `N >= 9`;
- finite mean and nonzero radius;
- sample covariance condition at most `MAX_SAMPLE_CONDITION`;
- finite positive-definite `Q` and positive `gamma`;
- correction condition at most `MAX_CORRECTION_CONDITION`;
- full-cache radial RMS at most `MAX_RADIAL_RMS`;
- finite sensor-unit hard- and soft-iron parameters.

The gravity surrogate changes the shared `theta`; there is no separate refined candidate or radial-regression allowance.
Published parameters change atomically only after all common checks pass.

Before first publication, return the existing calibration error for a full but invalid optimizer state. After first
publication, preserve the current behavior: suppress non-insufficient candidate errors and use the last published
correction. Never return a successful correction before the cache is full.

## Complexity

Let `B` be minibatch size.

- Online gradient: `O(10 B)`.
- Candidate conversion: fixed-size `3 x 3` decompositions.
- Full publication validation: `O(N)`.
- Existing diversity-cache maintenance: expected `O(N)` for a full cache.
- Additional persistent optimizer state: `O(1)`.

The whole call remains `O(N)` because cache maintenance and validation are linear. The change removes both per-sample
`9 x 9` normal-matrix accumulations and both `9 x 9` Cholesky solves.

## Implementation sequence

1. Capture fixed-seed baseline timing and accuracy from `tests/mag_calibration_dummy.rs`.
2. Refactor cache mutation to expose the accepted row and cache-change state privately.
3. Add normalization state, rebasing, deterministic sampling, and `minibatch_size`.
4. Add the joint analytic gradient and transactional normalized-SGD update.
5. Remove the direct ellipsoid and affine gravity normal solves.
6. Keep candidate conversion, common quality gates, and last-known-good publication.
7. Update unit tests and algorithm documentation.
8. Run the same fixed-seed benchmark, compare results, and tune bounded optimizer constants if needed.

## Test plan

- Preserve all neighbor-cache invariant and expiry tests.
- Verify current-sample inclusion after append, replacement, and diversity rejection.
- Verify invalid samples do not update optimizer state.
- Verify deterministic streams produce identical parameter and correction sequences.
- Verify `minibatch_size` boundary behavior.
- Compare analytic radial and gravity gradients against finite differences.
- Verify normalization rebasing preserves the represented ellipsoid and gravity projection.
- Preserve full-SPD and asymmetric-coverage direction error below `0.05`.
- Preserve repeated-correction stability below `0.01`.
- Preserve degenerate-sample rejection and last-known-good fallback.
- Preserve invalid-gravity equivalence and verify zero gravity weight follows the magnetometer-only path.
- Verify consistent gravity improves or at least does not degrade fixed synthetic and dummy accuracy.
- Compare online objective and direction accuracy with a test-only direct-solver oracle if convergence tuning requires
  it.

## Benchmark and acceptance criteria

Use the deterministic `regression` cases from `tests/mag_calibration_dummy.rs` both before and after implementation.
Record:

- average `evaluate_correct` computation time;
- average and worst successful angular error;
- average and worst post-warm-up angular error;
- count and wall time until first successful publication.

The implementation is accepted when:

- both production `9 x 9` solves are removed;
- existing unit tests and the `18 degree` post-warm-up integration limit pass;
- fixed-seed accuracy is materially comparable to the direct-solver baseline;
- first publication remains quality-gated and occurs only after a full cache;
- optimizer failures cannot corrupt published state;
- per-call fitting work is bounded by `minibatch_size`;
- no runtime dependency, asynchronous task, or public error variant is added.

## Validation commands

From `__module~/driver`:

```bash
cargo test --package ar-drivers --no-default-features --lib fusion::mag_calibration_test
cargo test --package ar-drivers --no-default-features --lib fusion::naive_cf_test
cargo test --package ar-drivers --no-default-features --test mag_calibration_dummy regression -- --nocapture
cargo fmt --all -- --check
cargo check --all-targets --all-features
cargo test --all-targets --all-features
cargo clippy --all-targets --all-features -- -D warnings
```
