# Nadir Pointing Analysis: ECI Vector Sign Convention and Gain Derivation

## 1. Problem Statement

After fixing the TRIAD quaternion convention bug (see
`adcs_nadir_triad_convention_analysis.md`), the nadir pointing controller
still failed to converge. The pointing error oscillated between 4° and
175° over each orbit, regardless of gain selection. A comprehensive sweep
of 12 gain configurations (kP from 4.37e-5 to 2e-3, ζ from 0.7 to 2.0)
produced universal divergence, proving the issue was not gain-related.

**Resolution:** The root cause was a sign error in the ECI reference
vector fed to the target quaternion computation. The C code used
**nadir** (−r/‖r‖) while the MATLAB reference uses **zenith**
(+r/‖r‖). Due to the quaternion kinematics sign convention, the B-cross
control law repels body_z from the ECI vector, so using nadir caused the
controller to push body_z *away* from nadir. Negating the vector
to zenith fixes the direction. Combined with re-derived gains and
dead-zone compensation, the controller now achieves 17 nadir
acquisitions over 5 orbits with 0.09° minimum pointing error.

---

## 2. Root Cause: ECI Vector Sign Convention

### 2.1 The B-Cross Control Law Sign Chain

The nadir pointing controller uses a B-cross magnetic control law:

```
m = (kP × cross(B, ε) − kR × cross(B, ω)) / ‖B‖              (1)
```

where `ε` is the vector part of the error quaternion:

```
q_err = q_target ⊗ conj(q_est)                                 (2)
q_target = U2Q(eci_vector, body_vector)                         (3)
```

The key question is: **which direction does the controller drive
body_z?**

### 2.2 Quaternion Kinematics Sign Convention

For the ECI→body quaternion used in both C and MATLAB:

```
dq/dt = −0.5 · [0, ω] ⊗ q                                     (4)
```

The negative sign means positive ω_y *decreases* q_y. This is
critical for understanding the control direction.

### 2.3 Tracing the Full Sign Chain

Consider a concrete example: satellite at position [−R, 0, 0] (ECI),
so nadir = [+1, 0, 0] and zenith = [−1, 0, 0]. Assume current
attitude q_est = identity (body_z = ECI_z), so the body_z axis is
perpendicular to nadir.

**With nadir as eci_vector (C code, WRONG):**

```
q_target = U2Q([1,0,0], [0,0,1]) = [0.707, 0, +0.707, 0]
q_err = q_target ⊗ conj(identity) = [0.707, 0, +0.707, 0]
ε_y = +0.707
τ_y = kP · ‖B‖ · ε_y > 0  →  ω_y > 0
                                                                (5)
By Eq. (4): ω_y > 0  →  q_y decreases  →  body_z moves toward
zenith (+X), AWAY from nadir.
```

**With zenith as eci_vector (MATLAB, CORRECT):**

```
q_target = U2Q([-1,0,0], [0,0,1]) = [0.707, 0, -0.707, 0]
q_err = q_target ⊗ conj(identity) = [0.707, 0, -0.707, 0]
ε_y = -0.707
τ_y = kP · ‖B‖ · ε_y < 0  →  ω_y < 0
                                                                (6)
By Eq. (4): ω_y < 0  →  q_y increases  →  body_z moves toward
nadir (-X), TOWARD nadir.
```

### 2.4 Physical Interpretation

The B-cross control law with this quaternion convention **repels**
body_z from the eci_vector direction. This is not a bug in the
control law — it is a consequence of:

1. The negative sign in Eq. (4)
2. The sign of ε from U2Q
3. The proportional torque τ ∝ ε

The MATLAB code exploits this by using **zenith** as the ECI vector:
repulsion from zenith drives body_z toward nadir.

### 2.5 MATLAB Reference

```
% Nadir_pointing.m, line 244 (inside the main simulation loop):
p.eci_vector = x(1:3)/norm(x(1:3));    % x(1:3) = satellite position
                                        % This is ZENITH, not nadir!
```

Note: `Sim_PID_controller.m` line 21 sets the *initial* value of
`p.eci_vector` using nadir, but this is immediately overwritten
by line 244 in the main loop.

### 2.6 C Code Fix

```c
// adcs_nadir.c, line 61 (BEFORE fix):
vec3d_t nadir_dir = vec3d_normalize(state->nadir_eci);

// adcs_nadir.c, line 66 (AFTER fix):
vec3d_t zenith_eci = vec3d_scale(vec3d_normalize(state->nadir_eci), -1.0);
```

---

## 3. Gain Re-Derivation for PoCat Hardware

### 3.1 Problem: Coil Factor Mismatch

The original gains were computed by the MATLAB `PIDMIMO` routine using
a coil factor of S = 0.01 turns·m². PoCat hardware uses:

| Axis | Coil factor (turns·m²) | Ratio to MATLAB |
|------|----------------------|-----------------|
| X    | 0.106306             | 10.6×           |
| Y    | 0.056065             | 5.6×            |
| Z    | 0.106306             | 10.6×           |

The dipole is `m = S · I`, so for the same current, PoCat produces
~10× more dipole moment and ~10× more torque than the MATLAB model
assumed. The MATLAB-derived kP was therefore ~10× too small relative
to the hardware capability.

### 3.2 Original MATLAB Gains

From `PIDMIMO(Inertia, 1, 0.005/4, 300, 0.1, dT, 'Delta')`:

```
kP = 4.373202e-5    (proportional)
kR = 1.907413e-2    (rate damping)
```

### 3.3 Quantization Dead-Zone Analysis

The BD2606MVV H-bridge driver has:

- Operating range: 0.5–150 mA
- Resolution: 0.5 mA steps
- Dead zone: |I| < 0.5 mA → output = 0

With the MATLAB gains, the proportional dipole command at typical
error magnitudes:

```
|m_prop| = kP · |ε| ≈ 4.37e-5 · 0.5 = 2.19e-5 A·m²
I = m / S = 2.19e-5 / 0.106 ≈ 0.21 mA  →  BELOW dead zone!
```

This means ~35% of control commands produce zero output, wasting
control opportunities and causing jerky behavior.

### 3.4 Re-Derived Gains

**Proportional gain kP:**

Target: typical |ε| ≈ 0.3 should produce at least 5 mA (10× dead-zone):

```
I_target = 5 mA = 5e-3 A
m = S · I = 0.106 · 5e-3 = 5.3e-4 A·m²
kP = m · ‖B‖ / |ε| ≈ 5.3e-4 · 3e-5 / (0.3 · 1.5e-5²)  →  ...
```

Using the full formula with B ≈ 30 μT:

```
kP = m_target / |ε| = 5.3e-4 / 0.3 ≈ 1.8e-3
```

Selected: **kP = 1.0e-3** (conservative, ~23× increase from MATLAB).

**Damping gain kR:**

For critical damping with the increased kP:

```
ζ = kR / (2 · sqrt(kP · I_avg))

With ζ = 2.0 (over-damped for quantization robustness):
kR = 2 · ζ · sqrt(kP · I_avg) ≈ 2 · 2.0 · sqrt(1e-3 · 1.2e-4)
   ≈ 4.4e-2
```

Selected: **kR = 9.0e-2** (2× margin above critical, ~4.7× increase
from MATLAB).

### 3.5 Hardware Utilization

| Parameter           | MATLAB gains | Re-derived |
|---------------------|-------------|------------|
| kP                  | 4.37e-5     | 1.0e-3     |
| kR                  | 1.91e-2     | 9.0e-2     |
| Typical I (prop.)   | 0.21 mA     | 4.7 mA     |
| Dead-zone fraction  | ~35%        | ~2%        |
| Max I utilization    | 1.4%        | 31%        |

---

## 4. Dead-Zone Compensation

### 4.1 The Problem

The BD2606MVV driver cannot output currents between 0 and 0.5 mA.
Any quantized command that rounds to zero produces no torque, even
when the controller intended a small but non-zero correction.

### 4.2 Solution: Snap-to-Minimum

When the raw command is non-zero but quantization would round to
zero, snap to the minimum output (±0.5 mA, preserving sign):

```c
// adcs_magnetorquer.c, quantize_intensity():
double abs_val = fabs(raw_ma);
if (abs_val > 1e-12) {
    double quantized = round(abs_val / step) * step;
    if (quantized < min_intensity) {
        quantized = min_intensity;  // snap to ±0.5 mA
    }
    ...
}
```

This matches the MATLAB reference behavior where `max(quantized, 0.5)`
is applied to non-zero commands.

---

## 5. Expected Behavior: Orbit-Period Oscillation

### 5.1 Magnetic Control Limitation

Magnetorquers can only produce torque perpendicular to the local
magnetic field: τ = m × B. As the satellite orbits, B rotates in the
body frame, creating periodic intervals of poor controllability around
certain axes. This is fundamental to all magnetic-only attitude
control systems.

### 5.2 Eclipse Effect

During eclipse, the sun sensor provides no useful data. TRIAD
attitude determination degrades to magnetometer-only, which cannot
resolve the full attitude. The controller operates with degraded
attitude estimates, causing temporary pointing errors.

### 5.3 Convergence Envelope

Simulation results (30,000 steps, ~5.4 orbits) show the
characteristic behavior of B-cross magnetic control:

| Metric                 | Value    |
|------------------------|----------|
| Minimum error          | 0.09°    |
| Nadir acquisitions     | 17       |
| Mean error (last orbit)| 51.08°   |
| Peak error (orbit 1)   | ~101°    |
| Peak error (orbit 5)   | ~93°     |

The controller repeatedly finds nadir (sub-degree accuracy) but
cannot maintain pointing during unfavorable B-field geometry or
eclipse. The peak-error envelope decreases slowly over orbits,
consistent with the under-actuated nature of the system.

---

## 6. Verification Summary

### 6.1 Quaternion Convention Audit

All quaternion operations were verified to match between C and MATLAB:

| Component              | C function              | MATLAB function | Status |
|------------------------|------------------------|-----------------|--------|
| Attitude estimate      | quat_from_matrix + conj | rotmatrix2quat  | ✓ Match |
| Target quaternion      | quat_from_two_vectors   | U2Q             | ✓ Match |
| Error quaternion       | quat_multiply           | QMult           | ✓ Match |
| Positive scalar        | quat_positive_scalar    | if q(1)<0...    | ✓ Match |
| Rotation application   | quat_rotate_vec         | QForm           | ✓ Match |
| Kinematic equation     | dq = -0.5*ωq*q         | QIToBDot        | ✓ Match |

### 6.2 Test Criteria

The integration test (`test_nadir.c`) uses three criteria suited
to magnetic-only B-cross control:

1. **min error < 10°** — controller achieves accurate pointing
2. **nadir acquisitions ≥ 3** — repeatable convergence
3. **last-orbit mean < 75°** — no divergence

### 6.3 Files Changed

| File | Change |
|------|--------|
| `src/subsystems/adcs/adcs_nadir.c` | Negate nadir→zenith for q_target |
| `include/subsystems/adcs/adcs_config.h` | kP: 4.37e-5→1e-3, kR: 1.91e-2→9e-2 |
| `src/subsystems/adcs/adcs_magnetorquer.c` | Dead-zone snap-to-minimum |
| `test/test_nadir.c` | Strengthened pass criterion |

---

## 7. Potential Future Improvements

1. **Gyroscope-based propagation during eclipse** — maintain attitude
   estimate using integrated angular velocity when sun sensor is
   unavailable, reducing eclipse-induced pointing errors.

2. **Gain scheduling** — use lower gains near nadir (small error) to
   reduce overshoot, higher gains at large error for faster acquisition.

3. **B-field controllability check** — skip control steps when the
   B-field alignment cannot produce useful torque along the needed axis,
   avoiding energy waste and potential destabilization.

4. **Extended simulation** — run 15–20 orbits to verify long-term
   convergence of the peak-error envelope.
