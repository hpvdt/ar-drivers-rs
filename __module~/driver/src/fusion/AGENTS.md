# AGENTS.md - Fusion Module Guide

## Conventions

### Reference Frames

The library uses multiple coordinate reference frames:

- **RUB (Right-Up-Back)**: Android sensor coordinate system (used in raw sensor data)
- **FRD (Forward-Right-Down)**: Aerospace standard frame (used in fusion outputs)
- **Custom frames**: Configurable via AHRS for different applications

### Coordinate Transformations

- Treat the shared sensor-event documentation and the fusion module as the source
  of truth for reference frames and units. Device events currently use RUB, while
  fusion state and outputs use FRD.
- Keep frame transformations explicit and centralized, document device-specific
  deviations, and use the repository's existing linear-algebra types.

## Source Files

### Magnetometer Calibration (`mag_calibration.rs`)

`MagCalibrator<N>` keeps a cache of finite, nonzero FRD magnetometer samples. Old
samples expire according to `max_sample_lifespan_us`; after the cache is full, a
k-nearest-neighbor diversity heuristic decides whether a new sample should replace
an existing one. Calibration requires `N.max(9)` retained samples, which means the
entire cache must be populated and `N` must be at least 9.

Let $b$ denote the hard-iron offset, $D$ the symmetric positive-definite
soft-iron distortion matrix, $A$ the corresponding soft-iron correction matrix,
and $m_i$ an ideal unit-length magnetic-field sample. Their physical relationship
is

$$
x_i=b+Dm_i,
\qquad
\left\|m_i\right\|=1,
\qquad
m_i=A(x_i-b),
\qquad
AD=DA=I.
$$

The solver does not optimize $D$ or compute a matrix inverse. It represents $A$
directly with a lower-triangular factor $L$:

$$
A=L^TL.
$$

The optimized calibration state consists only of $b$ and the six direct
coordinates of $L$:

$$
[L_{00},L_{10},L_{11},L_{20},L_{21},L_{22}].
$$

Correction therefore uses only two matrix-vector multiplications:

$$
y=L^TL(x-b).
$$

The solver minimizes the regularized radial least-squares objective

$$
J(b,L)
= \frac{1}{2n}\sum_i
  \left(\left\|L^TL(x_i-b)\right\|-1\right)^2
  +  \frac{\lambda}{2}\left\|L\right\|_F^2,
\qquad \lambda=10^{-4}.
$$

The objective is differentiated but never evaluated. On the first calibration,
$b$ starts at the sample mean. Define

$$
\rho = \left(\frac{1}{n}\sum_i\left\|x_i-b\right\|^2\right)^{1/4};
$$

when $\rho$ is finite and nonzero, $L$ starts as $\rho^{-1}I$. Later
calibrations warm-start from the saved state.

Every calibration runs exactly 20 alternating sweeps:

1. Scan all samples once to accumulate the three offset-gradient coordinates and
   their diagonal Gauss-Newton curvatures, then update the three coordinates.
2. Scan all samples once to accumulate the six factor-gradient coordinates and
   their diagonal Gauss-Newton curvatures. Add $\lambda L$ to the averaged
   gradient and $\lambda$ to the averaged curvature, then update all six factor
   coordinates directly.

Each coordinate step is clamped to $[-0.25, 0.25]$. The coordinates of $L$,
including its diagonal, are unconstrained: the solver does not enforce the
positive-diagonal convention that would make the triangular factor unique. There
is no logarithmic parameterization, determinant or condition-number gate,
backtracking, convergence test, or final RMS validation. Frobenius regularization
is the only soft pressure against excessively large factor entries.
Non-finite sample contributions, deltas, and candidates are skipped; the previous
coordinate is retained. Once enough samples exist, the latest finite parameter
state is persisted and returned as a best-effort result.

With the sweep count fixed, calibration itself takes $O(n)$ time per call and
$O(1)$ auxiliary space. This excludes maintenance of the sample cache and its
k-nearest-neighbor replacement heuristic.
