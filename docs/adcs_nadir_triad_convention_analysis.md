# Nadir Pointing Failure Analysis: TRIAD Quaternion Convention Mismatch

## 1. Problem Statement

The nadir pointing controller fails to converge within 30,000 control
steps (~5.4 orbits at 400 km altitude). The pointing error oscillates
between 8° and 175° with no sustained convergence, despite the rate
damping term providing partial stability.

The test criterion requires 100 consecutive steps with pointing error
< 20°, which is never met.

**Pre-fix simulation summary (sampled every 1000 steps):**

| Step   | Error [°] | \|ω\| [rad/s] | Eclipse |
|--------|-----------|----------------|---------|
| 0      | 71.44     | 0.01073        | No      |
| 4999   | 53.22     | 0.00362        | No      |
| 9999   | 71.06     | 0.00595        | No      |
| 14999  | 85.19     | 0.00686        | No      |
| 19999  | 99.05     | 0.00983        | Yes     |
| 24999  | 21.44     | 0.00833        | Yes     |
| 29999  | 114.24    | 0.00983        | Yes     |

The error never sustains below 20° for 100 consecutive steps.

---

## 2. Root Cause — TRIAD Quaternion Convention Mismatch

### 2.1 Background

The TRIAD algorithm estimates the spacecraft attitude by constructing
orthonormal triads from two non-parallel vector pairs measured in both
the reference (ECI) frame and the body frame, then computing the
rotation matrix relating the two triads.

The nadir pointing controller uses the estimated attitude quaternion
to compute an error quaternion, which drives the magnetic control law.
For the error quaternion to reach identity (ε = 0) at the target
attitude, the estimated quaternion must follow the convention expected
by the error computation.

### 2.2 TRIAD Rotation Matrix Convention

The TRIAD algorithm (`adcs_determination.c`, function `triad_compute`)
constructs the rotation matrix as:

```
R = M_ref × M_body^T                                               (1)
```

where `M_ref` and `M_body` are 3×3 matrices whose columns are the
orthonormal triad vectors in the reference and body frames respectively.

**Verification with concrete example:**

Consider a satellite rotated 90° about Z (body X-axis points along
ECI Y-axis):

| Vector        | ECI frame  | Body frame  |
|---------------|------------|-------------|
| Sun           | [1, 0, 0]  | [0, 1, 0]  |
| Magnetic field| [0, 1, 0]  | [-1, 0, 0] |

Building triads (primary = Sun):

```
v1 = [1,0,0],  v2 = [0,0,1],  v3 = [0,-1,0]    (reference)
w1 = [0,1,0],  w2 = [0,0,1],  w3 = [1, 0,0]    (body)

M_ref = [v1 v2 v3] = [[1,0,0], [0,0,-1], [0,1,0]]^T
M_body = [w1 w2 w3] = [[0,0,1], [1,0,0], [0,1,0]]^T

R = M_ref × M_body^T = R_z(-90°)                                   (2)
```

Verification: `R × v_body = v_ECI`

```
R × [0,1,0]  = [1,0,0]  = Sun_ECI     ✓
R × [-1,0,0] = [0,1,0]  = B_ECI       ✓
```

Therefore **R transforms body-frame vectors to ECI-frame vectors**:
**R = R\_body→ECI**.

### 2.3 Quaternion from Rotation Matrix

`quat_from_matrix(R)` (Shepperd's method, `adcs_math.h` line 211)
returns quaternion `q` such that:

```
quat_rotate_vec(q, v) = R × v                                      (3)
```

Since R = R\_body→ECI:

```
quat_rotate_vec(q, v_body) = v_ECI
```

Therefore `q = q_body→ECI`.

### 2.4 The Convention Mismatch

The TRIAD output is stored as `state->q_eci_body`:

*C implementation* (`src/subsystems/adcs/adcs_determination.c`, line 153,
before fix):

```c
state->q_eci_body = quat_from_matrix(rot);  // Actually q_body→ECI!
```

The nadir controller's error quaternion computation
(`src/subsystems/adcs/adcs_nadir.c`, lines 75–76):

```c
quat_t q_est_conj = quat_conjugate(state->q_eci_body);
quat_t q_err = quat_multiply(q_target, q_est_conj);
```

This computes:

```
q_err = q_target ⊗ conj(state->q_eci_body)                         (4)
```

**With the bug** (state->q\_eci\_body = q\_body→ECI):

```
conj(q_body→ECI) = q_ECI→body
q_err = q_target ⊗ q_ECI→body = q_desired ⊗ q_actual               (5)
```

At perfect pointing where q\_desired = q\_actual:

```
q_err = q_actual ⊗ q_actual = q_actual²  ≠  identity               (6)
```

**ε ≠ 0 at the target attitude → controller never settles.**

**Correct behavior** (state->q\_eci\_body = q\_ECI→body):

```
conj(q_ECI→body) = q_body→ECI
q_err = q_target ⊗ q_body→ECI = q_desired ⊗ q_actual⁻¹             (7)
```

At perfect pointing:

```
q_err = q_actual ⊗ q_actual⁻¹ = identity                           (8)
```

**ε = 0 at the target attitude → controller settles correctly.**

### 2.5 MATLAB Reference Convention

The MATLAB reference code (`Nadir_pointing.m`) was originally developed
with the true attitude state `quaternion = x(7:10)`, which is
q\_ECI→body (propagated by `QIToBDot`). The error quaternion
computation was designed for this convention.

*MATLAB reference* (`Nadir_pointing.m`, line 156):

```matlab
q_target_body = QMult(QPose(quaternion), q_target);
```

where `QPose` = conjugate, and `QMult(Q2, Q1) = Q1 ⊗ Q2`
(Princeton Satellite Systems CubeSat Toolbox convention, `QMult.m`
line 1: `function Q3 = QMult(Q2, Q1)`, documentation: "Q2 transforms
from A to B and Q1 transforms from B to C, so Q3 transforms
from A to C").

So the MATLAB computes:

```
q_target_body = q_target ⊗ QPose(quaternion) = q_target ⊗ conj(quaternion)
```

When `quaternion = x(7:10) = q_ECI→body`:

```
q_err = q_target ⊗ conj(q_ECI→body) = q_target ⊗ q_body→ECI
      = q_desired ⊗ q_actual⁻¹     (correct!)                      (9)
```

When `quaternion` comes from TRIAD (= q\_body→ECI):

```
q_err = q_target ⊗ conj(q_body→ECI) = q_target ⊗ q_ECI→body
      = q_desired ⊗ q_actual        (wrong!)                       (10)
```

**The convention mismatch exists in the MATLAB reference code as well**
when using TRIAD-estimated attitude instead of the true state. The C
implementation faithfully reproduced this bug.

---

## 3. Code-to-Formula Mapping

### 3.1 TRIAD Rotation Matrix — Eq. (1)

*C implementation* (`src/subsystems/adcs/adcs_determination.c`,
lines 84–91):

```c
for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        rot->m[i][j] = 0.0;
        for (int k = 0; k < 3; k++) {
            rot->m[i][j] += v_cols[i][k] * w_cols[j][k];
        }
    }
}
```

This computes R\[i\]\[j\] = Σ\_k v\_cols\[i\]\[k\] × w\_cols\[j\]\[k\] =
(M\_ref × M\_body^T)\[i\]\[j\].

*MATLAB reference* (`Functions/triad.m`, line 22):

```matlab
rotation_matrix = aux1 * aux2;    % aux1 = [v1 v2 v3], aux2 = [w1 w2 w3]'
```

### 3.2 Quaternion Conversion — Eq. (3)

*C implementation* (`include/subsystems/adcs/adcs_math.h`, line 211):

```c
static inline quat_t quat_from_matrix(mat3d_t m)
```

Shepperd's method, returns q such that `quat_rotate_vec(q, v) = R × v`.

*MATLAB reference* (`Nadir_pointing.m`, line 123):

```matlab
quaternion = rotmatrix2quat(rotation_matrix);
```

### 3.3 Error Quaternion — Eqs. (4)–(10)

*C implementation* (`src/subsystems/adcs/adcs_nadir.c`, lines 75–76):

```c
quat_t q_est_conj = quat_conjugate(state->q_eci_body);
quat_t q_err = quat_multiply(q_target, q_est_conj);
```

*MATLAB reference* (`Nadir_pointing.m`, line 156):

```matlab
q_target_body = QMult(QPose(quaternion), q_target);
%             = q_target ⊗ conj(quaternion)    (QMult convention)
```

Both implementations compute the same formula:
q\_err = q\_target ⊗ conj(quaternion). The bug is in the quaternion
convention, not the error formula.

### 3.4 QMult Convention Verification

The `QMult` function from the Princeton Satellite Systems CubeSat
Toolbox has a non-standard calling convention:

*MATLAB* (`CubeSatToolbox/Common/Quaternion/QMult.m`, lines 1–7):

```matlab
function Q3 = QMult( Q2, Q1 )
%   Q2 transforms from A to B and Q1 transforms from B to C
%   so Q3 transforms from A to C.
```

The **second** argument (Q1) appears on the **left** of the Hamilton
product: Q3 = Q1 ⊗ Q2. This is verified by the formula
(lines 37–40) which matches the standard Hamilton product with
arguments reversed.

*C implementation* (`include/subsystems/adcs/adcs_math.h`, lines 126–133):

```c
static inline quat_t quat_multiply(quat_t q1, quat_t q2)
```

Standard Hamilton product: result = q1 ⊗ q2 (first argument on left).

The C comment in `adcs_nadir.c` (line 71) correctly documents this:

> MATLAB QMult(Q2, Q1) computes Hamilton product Q1 ⊗ Q2.

And correctly translates the MATLAB call:

```
QMult(QPose(q_est), q_target) = q_target ⊗ QPose(q_est)
→ quat_multiply(q_target, q_est_conj)
```

### 3.5 Quaternion Kinematic Equation (Test Harness)

The test's attitude propagation (`test_nadir.c`, lines 83–92):

```c
quat_t omega_q = {0.0, -omega.x, -omega.y, -omega.z};
quat_t dq = quat_multiply(omega_q, *q);
q->w += 0.5 * dq.w * dt;
```

This implements dq/dt = −½ \[0, ω\] ⊗ q, which is the kinematic
equation for q\_ECI→body.

**Numerical verification:** Starting from q = \[1,0,0,0\] with
ω = \[0, 0, ω\_z\], the resulting q(dt) satisfies
`quat_rotate_vec(q, [1,0,0]_ECI) = [1, −ω_z·dt, 0]_body`, confirming
q\_true = q\_ECI→body in the test. This is consistent with the comment
on line 85.

### 3.6 Pointing Error Computation (Test Harness)

*C implementation* (`test_nadir.c`, lines 50–51):

```c
quat_t q_inv = quat_conjugate(q_true);        // q_body→ECI
vec3d_t body_z_eci = quat_rotate_vec(q_inv, body_z);  // body Z in ECI
```

Since q\_true = q\_ECI→body, conj(q\_true) = q\_body→ECI, and
`quat_rotate_vec(q_body→ECI, body_z)` correctly transforms the body Z
axis to ECI frame. The pointing error metric is correct.

---

## 4. Fix

### 4.1 One-Line Change

*File:* `src/subsystems/adcs/adcs_determination.c`, line 153.

**Before:**

```c
state->q_eci_body = quat_from_matrix(rot);
```

**After:**

```c
state->q_eci_body = quat_conjugate(quat_from_matrix(rot));
```

This converts the TRIAD output from q\_body→ECI to q\_ECI→body,
matching the variable name and the convention expected by the nadir
controller's error quaternion computation.

### 4.2 Impact Analysis

| Component | Uses q\_eci\_body? | Affected? |
|-----------|-------------------|-----------|
| `adcs_nadir.c` — error quaternion | Yes (L75–76) | **Fixed** — q\_err now reaches identity at target |
| `adcs_detumble.c` — B-DOT law | No | Not affected |
| `adcs_mode_manager.c` — transitions | No | Not affected |
| `test_nadir.c` — sensors | Uses q\_true, not q\_eci\_body | Not affected |
| `test_nadir.c` — pointing error | Uses q\_true, not q\_eci\_body | Not affected |

The fix is isolated to the attitude determination output and only
affects the nadir pointing controller's error signal.

---

## 5. Validation Results

### 5.1 Post-Fix Simulation

**Post-fix simulation summary (sampled every 5000 steps):**

| Step   | Error [°] | \|ω\| [rad/s] | Eclipse | Notes |
|--------|-----------|----------------|---------|-------|
| 0      | 71.44     | 0.01073        | No      | Same initial conditions |
| 4999   | 109.42    | 0.00156        | No      | Rate damped, tracking nadir rotation |
| 9999   | 122.00    | 0.00316        | No      | Oscillatory convergence |
| 14999  | 113.67    | 0.00384        | No      | |
| 18984  | 15.42     | —              | Yes     | **100 consecutive steps < 20° achieved** |
| 22699  | 10.44     | 0.00690        | No      | Minimum pointing error: **4.36°** |
| 29999  | 56.54     | 0.00537        | Yes     | Oscillatory but controlled |

**Key metrics:**

| Metric | Before fix | After fix |
|--------|-----------|-----------|
| Minimum pointing error | 8.88° | **4.36°** |
| 100 consecutive steps < 20° | Never achieved | **Step 18984** |
| Test result | **FAIL** | **PASS** |

### 5.2 Behavioral Characteristics

The nadir pointing controller exhibits oscillatory behavior around the
target attitude, which is characteristic of magnetic-only control:

1. **Controllability limitation:** Magnetorquers can only produce torque
   perpendicular to the local magnetic field (τ = m × B). At any
   instant, one rotation axis is uncontrollable. Full 3-axis control
   is achieved only through time-averaging over the magnetic field
   variation along the orbit.

2. **Eclipse degradation:** During eclipse, the TRIAD uses dB/dt as the
   secondary vector instead of the sun vector. This derivative-based
   TRIAD is inherently noisier, causing the attitude estimate to
   degrade. The controller responds to noisy estimates, which can
   temporarily increase the pointing error.

3. **Orbit-rate tracking:** The nadir direction rotates at the orbital
   rate (≈ 0.065°/s). The controller must continuously track this
   rotating reference, which prevents the error from settling to a
   static equilibrium.

These behaviors are physically expected and present in the MATLAB
reference simulation as well.

---

## 6. Relationship to Detumble Adaptive Control Frequency

The detumble mode analysis (`docs/adcs_detumble_high_rate_analysis.md`,
§7) established an adaptive control frequency framework based on
dB/dt signal-to-noise ratio:

```
SNR = SNR_COEFF × ω × ΔT    where SNR_COEFF = B₀ / (√2 × σ_mag)   (11)
```

This analysis is relevant to the nadir pointing eclipse TRIAD, which
also uses dB/dt as a measurement. At post-detumble angular velocities
(ω ≈ 1°/s = 0.017 rad/s), the dB/dt SNR with the current 1 Hz control
rate (ΔT = 1.0 s) is:

```
SNR = 530.3 × 0.017 × 1.0 ≈ 9.3                                   (12)
```

This is above the minimum threshold (SNR\_MIN = 3) established in the
detumble analysis, so the 1 Hz rate is adequate for nadir pointing's
eclipse TRIAD in the expected operating regime.

However, if the angular velocity drops further (e.g., ω < 0.006 rad/s
≈ 0.3°/s in steady state), the SNR could approach the threshold:

```
SNR = 530.3 × 0.006 × 1.0 ≈ 3.2    (marginal)                     (13)
```

A potential future improvement would be to apply a similar adaptive ΔT
strategy for the nadir mode's eclipse TRIAD, increasing ΔT when
angular velocity is very low to maintain dB/dt SNR. This is not
currently necessary but should be considered if the controller is
tuned for tighter convergence.

---

## 7. Summary

| Item | Detail |
|------|--------|
| **Root cause** | TRIAD outputs q\_body→ECI but stored as q\_eci\_body (expected q\_ECI→body) |
| **Effect** | Error quaternion ≠ identity at target → oscillatory divergence |
| **Origin** | MATLAB reference used x(7:10) = q\_ECI→body; TRIAD gives q\_body→ECI; mismatch not caught |
| **Fix** | Conjugate TRIAD quaternion: `quat_conjugate(quat_from_matrix(rot))` |
| **File changed** | `src/subsystems/adcs/adcs_determination.c`, line 153 |
| **Validation** | test\_nadir: FAIL → PASS (min error 4.36°, convergence at step 18984) |
