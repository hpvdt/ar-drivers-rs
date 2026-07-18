# AGENTS.md - Fusion Module Guide

## Reference Frames

The library uses multiple coordinate reference frames:

- **RUB (Right-Up-Back)**: Android sensor coordinate system (used in raw sensor data)
- **FRD (Forward-Right-Down)**: Aerospace standard frame (used in fusion outputs)
- **Custom frames**: Configurable via AHRS for different applications

## Coordinate Transformations

- Treat the shared sensor-event documentation and the fusion module as the source
  of truth for reference frames and units. Device events currently use RUB, while
  fusion state and outputs use FRD.
- Keep frame transformations explicit and centralized, document device-specific
  deviations, and use the repository's existing linear-algebra types.

## Magnetometer Calibration

`MagCalibrator<N>` keeps a cache of finite, nonzero FRD magnetometer samples. Old
samples expire according to `max_sample_lifespan_us`; after the cache is full, a
k-nearest-neighbor diversity heuristic decides whether a new sample should replace
an existing one. Calibration requires `N.max(9)` retained samples, which means the
entire cache must be populated and `N` must be at least 9.

The calibration state consists of a hard-iron offset $b$ and a directly stored
lower-triangular factor $F$ with coordinates
$[F_{00}, F_{10}, F_{11}, F_{20}, F_{21}, F_{22}]$. A raw sample $x$ is corrected by

$$
y = F^{-T}F^{-1}(x-b).
$$

The solver minimizes the regularized radial least-squares objective

$$
J(b,F)
= \frac{1}{2n}\sum_i
  \left(\left\|F^{-T}F^{-1}(x_i-b)\right\|-1\right)^2
  + \frac{\lambda}{2}\left\|F\right\|_F^2,
\qquad \lambda=10^{-4}.
$$

The objective is differentiated but never evaluated. On the first calibration,
$b$ starts at the sample mean and $F$ starts as $sI$, where $s$ is the fourth
root of the mean squared distance from that mean. Later calibrations warm-start
from the saved state.

Every calibration runs exactly 20 alternating sweeps:

1. Scan all samples once to accumulate the three offset-gradient coordinates and
   their diagonal Gauss-Newton curvatures, then update the three coordinates.
2. Scan all samples once to accumulate the six factor-gradient coordinates and
   their diagonal Gauss-Newton curvatures. Add $\lambda F$ to the averaged
   gradient and $\lambda$ to the averaged curvature, then update all six factor
   coordinates directly.

Each coordinate step is clamped to $[-0.25, 0.25]$. Factor diagonals may be
positive or negative: there is no logarithmic parameterization, determinant or
condition-number gate, backtracking, convergence test, or final RMS validation.
Frobenius regularization is the only soft pressure against excessively large
factor entries. Non-finite sample contributions, deltas, and candidates are
skipped; the previous coordinate is retained. Once enough samples exist, the
latest finite parameter state is persisted and returned as a best-effort result.

With the sweep count fixed, calibration itself takes $O(n)$ time per call and
$O(1)$ auxiliary space. This excludes maintenance of the sample cache and its
k-nearest-neighbor replacement heuristic.
