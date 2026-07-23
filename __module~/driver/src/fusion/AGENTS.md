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

The solver estimates the ellipsoid as a quadratic form. Define the sample mean,
RMS radius, and dimensionless samples as

$$
\mu=\frac{1}{n}\sum_i x_i,
\qquad
r=\sqrt{\frac{1}{n}\sum_i\left\|x_i-\mu\right\|^2},
\qquad
u_i=\frac{x_i-\mu}{r}.
$$

Centering and scaling keep the direct solve independent of the sensor units and
reduce its numerical condition. A non-finite or zero $r$ is rejected. The
condition number of the centered sample covariance must not exceed $10^2$.

Let $Q$ be a symmetric ellipsoid shape matrix and $q$ its linear term.
Substituting the physical model $x_i=b+Dm_i$ with $A=D^{-1}$ into the
normalization $u_i=(x_i-\mu)/r$ and expanding $\left\|A(x_i-b)\right\|^2=1$
shows their physical content:

$$
Q=\gamma r^2A^2,
\qquad
q=-2Qd,
\qquad
d=\frac{b-\mu}{r},
\qquad
\gamma=1+d^TQd,
$$

so $Q$ is the squared correction matrix in normalized units, $d$ is the
normalized hard-iron center, and $\gamma$ the ellipsoid scale. The normalized
samples obey

$$
u_i^T Q u_i + q^T u_i = 1.
$$

The six independent coordinates of $Q$ and the three coordinates of $q$ form the
nine-parameter vector

$$
\theta=[Q_{00},Q_{11},Q_{22},Q_{01},Q_{02},Q_{12},q_0,q_1,q_2]^T.
$$

For

$$
\phi(u)=[u_x^2,u_y^2,u_z^2,2u_xu_y,2u_xu_z,2u_yu_z,u_x,u_y,u_z]^T,
$$

the solver minimizes the regularized algebraic least-squares objective

$$
J(\theta)
=\frac{1}{2n}\sum_i\left(\phi(u_i)^T\theta-1\right)^2
+\frac{\lambda}{2}\left\|Q-I\right\|_F^2,
\qquad \lambda=10^{-4}.
$$

The Frobenius regularization targets the identity shape — the exact fit for
ideal normalized samples — instead of the non-positive-definite zero matrix,
biasing candidates away from indefinite shapes while keeping the entire
objective quadratic. If $R=\operatorname{diag}(1,1,1,2,2,2,0,0,0)$ and
$e_d=[1,1,1,0,0,0,0,0,0]^T$, the unique candidate is obtained with one
direct linear solve:

$$
\left(\frac{1}{n}\sum_i\phi_i\phi_i^T+\lambda R\right)\theta
=\frac{1}{n}\sum_i\phi_i+\lambda e_d.
$$

There are no alternating sweeps, warm starts, parameter clamps, or convergence
iterations.

After solving, the normalized hard-iron center and ellipsoid metric are

$$
d=-\frac{1}{2}Q^{-1}q,
\qquad
\gamma=1+d^TQd,
\qquad
M=\frac{Q}{\gamma}.
$$

The candidate requires finite coefficients, positive-definite $Q$, and positive
$\gamma$. Transforming back to sensor units gives

$$
b=\mu+rd,
\qquad
A=\frac{1}{r}M^{1/2}.
$$

The principal symmetric positive-definite square root is computed once from the
eigendecomposition of $M$. The correction matrix condition number must not exceed
$10$, and the RMS radial residual

$$
\sqrt{\frac{1}{n}\sum_i\left(\left\|A(x_i-b)\right\|-1\right)^2}
$$

must not exceed $0.1$. Before the first successful calibration, failed solves and
rejected candidates return a specific `BadCalibration`. Later rejected candidates
leave the persisted calibration unchanged and the reading uses that last accepted
state. An incomplete sample buffer still returns `InsufficientSamples`. Successful
calibration persists $b$ and $A$ directly; correcting a reading then requires one
matrix-vector multiplication:

$$
m=A(x-b).
$$

Accumulating the fixed $9\times9$ normal system and validating the candidate take
$O(n)$ time per calibration and $O(1)$ auxiliary space. This excludes maintenance
of the sample cache and its k-nearest-neighbor replacement heuristic.
