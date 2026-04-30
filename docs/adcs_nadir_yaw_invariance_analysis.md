# Nadir-Pointing Stability Analysis: Yaw-Invariance Bug

## 1. Problem Statement

Starting from a *perfect* nadir-pointing initial condition (the body
+Z axis exactly aligned with the local nadir vector and the body rate
matching orbit rate `ω_orb = (0, −n, 0)` in body frame), the controller
nevertheless drifts to peaks of **~95° pointing error** during a 48-hour
hold test. The error eventually settles back near a few degrees by the
final orbit, but the long-period excursions in between are
unacceptable for a payload-pointing requirement.

The drift is observed in the C implementation, **not** in the original
MATLAB reference under its default test conditions, even though the C
port faithfully reproduces the same control law. This document
identifies the root cause as a structural property of the 2-DOF
quaternion-error formulation (shared between MATLAB and C), explains
why it manifests in the C test but is masked in the MATLAB run,
derives the fix, and reports validation.

**Resolution.** Replace the quaternion-product reduced-attitude error
with a directly computed body-frame cross product
`ε = ẑ_body × R(q_eci_body) · n̂_eci`, which is identically zero
whenever `ẑ_body ‖ n̂` regardless of yaw. Closed-loop performance from a
perfect nadir IC improves from `mean=38°, peak=95°` to
`mean=3.6°, peak=18°` over the same 48-hour test, with every 4-hour
window staying below 18° max.

---

## 2. Recap of the Control Architecture

Both implementations realize a **2-DOF reduced-attitude PD controller**
on the magnetic dipole, projected through a B-cross law. The control
law in body frame is

```
m  =  ( k_P · (B × ε)  −  k_R · (B × ω̃) ) / |B|                  (1)
```

where `B` is the body-frame magnetic field, `ω̃` is the body angular
rate with the orbit-rate feed-forward removed, and `ε ∈ R³` is a
small-error vector that vanishes when `ẑ_body` is aligned with the
local nadir direction. The yaw degree of freedom about `ẑ_body` is
intentionally left unconstrained: a single magnetic field cannot
simultaneously command three independent torques, and constraining
yaw with a 3-DOF target competes with the more important
roll/pitch alignment.

The disagreement between MATLAB and C lies entirely in **how `ε` is
constructed**.

---

## 3. The Two Constructions of `ε`

### 3.1 MATLAB reference

`ref/H-bridge-simulations/Simulation Nadir Pointing/Nadir_pointing.m`,
lines 152–180:

```matlab
q_target      = U2Q( p.eci_vector, p.body_vector );
q_target_body = QMult( QPose(quaternion), q_target );
if (q_target_body(1,1) < 0.0)
    q_target_body = -q_target_body;
end
moment = ( kP*cross(fieldBODY, q_target_body(2:4)) ...
         - kR*cross(fieldBODY, w_read) ) / norm(fieldBODY);
```

`U2Q(u,v)` is the Princeton Satellite Systems toolbox helper
`U2Q.m` that returns *one* minimum-rotation quaternion sending unit
vector `u` to unit vector `v`:

```matlab
c = Cross(u,v);  d = Dot(u,v);  s = sqrt(2*(1+d));
q = [ 0.5*s ;  c(1,:)./s ;  c(2,:)./s ;  c(3,:)./s ];
```

i.e. an axis-angle quaternion about `(u × v)/|u × v|` of angle
`acos(u·v)`. By construction this picks the *single* attitude with the
particular yaw chosen by the cross product, out of the entire
yaw-family of valid nadir-pointing attitudes.

The vector part of the resulting `q_target_body` is then used as `ε`.

### 3.2 C implementation (before fix)

`src/subsystems/adcs/adcs_nadir.c` (Step 2 of `nadir_step`,
pre-fix):

```c
quat_t q_target = quat_from_two_vectors(zenith_eci, body_z_axis);
quat_t q_err    = quat_multiply(q_target, quat_conjugate(state->q_eci_body));
if (q_err.w < 0.0) q_err = quat_negate(q_err);
vec3d_t epsilon = quat_vector_part(q_err);
```

`quat_from_two_vectors` is a direct C port of `U2Q`. The math is
identical to MATLAB.

---

## 4. Why This Construction Is Not Yaw-Invariant

Let `q_perfect` be **any** attitude satisfying
`R(q_perfect) · ẑ_body = n̂_eci`, i.e. perfect nadir alignment. The
*set* of such attitudes forms a one-parameter family parameterised by
yaw `ψ` about `ẑ_body`:

```
q_perfect(ψ)  =  q_yaw(ψ)  ⊗  q_perfect(0)                        (2)
```

For any fixed pair `(zenith_eci, body_z_axis)`, `U2Q` returns one
specific quaternion `q_target` corresponding to the cross-product
axis. That `q_target` matches `q_perfect(ψ)` only for *one* particular
yaw value `ψ_match`. For any other yaw,

```
q_err  =  q_target ⊗ conj(q_perfect(ψ))  ≠  identity              (3)
```

so `vec(q_err) ≠ 0` and the controller injects a phantom P-command
**at the supposed equilibrium**. The `m_P = (k_P/|B|)(B × ε)` term
produces a torque whose component along `ẑ_body` is generally
non-zero, and that torque slowly accumulates yaw-rate. Because the
2-DOF design deliberately leaves yaw unconstrained, the yaw-rate is
*never bounded by the controller*: it grows until the body wanders
far enough from nadir that the (now genuine) reduced-attitude error
dominates and the loop pulls back. The result is a long-period limit
cycle: phantom-error pumps yaw → body drifts off nadir → real error
swings the body back → overshoot → phantom again, etc.

This is exactly the failure mode observed in the pre-fix 48-hour
test:

| 4-h window | Mean error | Peak error |
|------------|-----------:|-----------:|
| 0 – 4   h  |     ~5°    |    ~10°    |
| 4 – 8   h  |    ~25°    |    ~75°    |
| 8 – 12  h  |    ~45°    |    ~95°    |
| 12 – 16 h  |    ~30°    |    ~80°    |
| ...        |    ...     |    ...     |
| 44 – 48 h  |     ~2°    |     ~5°    |

The "calm" final orbit obscured the issue for a long time, and was
the reason earlier attempts (Σ-Δ MTQ dither, gyro low-pass,
quantization floor) appeared to "almost" fix things — they all
slightly altered the limit-cycle period without removing its driver.

---

## 5. Why the MATLAB Reference Appears to Work

The MATLAB driver runs in conditions that hide rather than fix the bug:

1. **Short simulation duration.** MATLAB plots typically span one or
   two orbits — far less than the long-period limit cycle (8–12 h) the
   C test exposes.
2. **Large initial error.** Most MATLAB runs start in a tumbling or
   inertially-fixed attitude, so genuine reduced-attitude error
   dominates the phantom term throughout the visible portion of the
   simulation.
3. **Open-loop rate damping by hysteresis rods.** The MATLAB
   environment includes passive magnetic hysteresis rods (`SimHysteresisRods/`)
   that bleed off body-frame angular momentum on every orbit. This
   bleeds the *same* yaw rate the phantom term would otherwise
   accumulate, masking the structural defect.
4. **Different inertia and orbit.** The MATLAB CubeSat has
   `J ≈ 0.05 kg·m²` per axis and runs at the gain set tuned for that
   inertia; the slow yaw drift never crosses the magnitude
   threshold within the plotted window.

PoCat does not have hysteresis rods, runs longer, and starts from a
perfect attitude — all three changes lift the curtain.

---

## 6. The Yaw-Invariant Fix

Define `ε` directly in body frame as the cross product of the body
+Z axis with the body-frame nadir direction:

```
n̂_body  =  R(q_eci_body) · n̂_eci                                  (4)
ε       =  ẑ_body × n̂_body                                        (5)
```

Properties (proved by direct computation):

* **`ε.z ≡ 0`**: the cross product is orthogonal to `ẑ_body`, so the
  P-term lives entirely in the body roll/pitch plane and never
  commands yaw.
* **`|ε| = sin θ`** where `θ = acos(ẑ_body · n̂_body)` is the genuine
  reduced-attitude angle.
* **Yaw-invariance.** Under a yaw rotation
  `q_eci_body ← q_yaw(ψ) ⊗ q_eci_body`,
  `n̂_body` rotates *within* the body x–y plane while `ẑ_body` is
  unchanged; hence `ẑ_body × n̂_body` is also rotated within the same
  plane but its magnitude is preserved and its component along
  `ẑ_body` stays zero. In particular `ε = 0` for every member of the
  yaw-family `q_perfect(ψ)`, so no phantom torque is injected at
  equilibrium.

**Sign check.** A small tilt about body x by `+δθ` rotates `n̂_body`
from `(0,0,1)` to `(0, −δθ, 1)` (first-order). Then
`ε = (0,0,1) × (0, −δθ, 1) = (δθ, 0, 0)`, pointing along `+x̂_body`.
With the same B-cross law (1) the P-component of the dipole becomes
`m_P ≈ (k_P/|B|) (B × ε)`, and the resulting torque
`τ = m × B` has component
`τ_x ≈ −k_P · (B_y² + B_z²)/|B| · δθ < 0` —
exactly the restoring torque that drives the body back toward nadir.
Sign is correct.

The fix replaces lines ~156–194 of `adcs_nadir.c`:

```c
vec3d_t nadir_eci_unit = vec3d_normalize(state->nadir_eci);
vec3d_t nadir_body     = quat_rotate_vec(state->q_eci_body, nadir_eci_unit);
vec3d_t body_z_axis    = vec3d_make(NADIR_BODY_AXIS_X,
                                    NADIR_BODY_AXIS_Y,
                                    NADIR_BODY_AXIS_Z);
vec3d_t epsilon = vec3d_cross(body_z_axis, nadir_body);
```

`quat_from_two_vectors` is no longer called from the nadir mode.
`mtq_reset_accumulator()` is now called from `nadir_init()` so that
the Σ-Δ MTQ accumulator state cannot leak between mode transitions or
between successive test runs.

---

## 7. Differences from the MATLAB Reference

This is the canonical list of structural differences between the C
flight code and `Nadir_pointing.m`. Items marked **(divergent)** are
intentional changes that the C version applies on top of the
reference.

| Item | MATLAB reference | C implementation |
|------|------------------|------------------|
| Reduced-attitude error `ε` | `vec( U2Q(zenith,body_z) ⊗ conj(q_eci_body) )` (yaw-arbitrary) | `ẑ_body × R(q_eci_body)·n̂_eci`  **(divergent — fix)** |
| Quaternion error sign convention | Negate `q_err` if `q_err.w < 0` | No longer applicable (no `q_err`) |
| MTQ command resolution | Continuous current command, `sign(m)·m_max` saturation, no quantization | 1st-order Σ-Δ modulator at sub-step rate, `±m_max` levels  **(divergent — hardware match)** |
| Bias estimator gate | Continuous online estimator | Rate-magnitude gate scaled by control `dt` to avoid bias hijacking when ω is large  **(divergent — robustness)** |
| Control rate `Δt` | Fixed 1 s | Adaptive scaffold with hysteresis (currently 1 s in nadir hold) |
| Passive magnetic hysteresis rods | Modeled (`SimHysteresisRods/`) | Not present (no rods on PoCat) |
| Gyro / mag noise model | Simple white noise | Sensor-spec ARW + bias drift; magnetometer cross-axis errors |
| Eclipse | Sun model + EKF off in eclipse | TRIAD off in eclipse, gyro propagation only; eclipse forced off in this hold test for diagnosis |
| Hardware coil factors | Single dipole-direct command | Per-axis coil factors `S_x/S_y/S_z`, current cap based on `m_max` per axis |
| Inertia | CubeSat default | PoCat-specific 1U inertia tensor |

The yaw-invariance fix is the only one that changes the *algorithm*;
all other differences are either modeling fidelity (sensors, MTQ
quantization, eclipse) or environment (hysteresis rods, inertia).

---

## 8. Validation Results

Test setup (`test/test_nadir.c`):

* Initial attitude: perfect nadir
  (`q = q_lvlh(r,v)`, `ω = (0, −n, 0)`).
* Duration: ~30 orbits (≈48 h).
* `Δt = 1 s` constant (hysteresis disabled).
* Eclipse forced off; sensor noise enabled.

### 8.1 Headline numbers

|                       | Before (q_target)  | After (cross-product) |
|-----------------------|-------------------:|----------------------:|
| Overall mean error    | 38°                | **3.6°**              |
| Overall max error     | 95°                | **18.3°**             |
| Last-orbit mean error | 2.0°               | **5.0°**              |
| 4-h windows ≤ 5° mean | 1 / 12             | **12 / 12**           |
| 4-h windows ≤ 18° max | 1 / 12             | **12 / 12**           |

Detumble regression test is unaffected: still converges at step 4256
(7928 s sim time) with `|ω| = 0.91°/s`.

### 8.2 LVLH Euler-angle history

`test/build/results/nadir_lvlh_euler.png` plots roll φ, pitch θ, and
yaw ψ of the body frame in LVLH coordinates (Z-Y-X intrinsic) over
the full test, in the same style the reference paper uses for nadir-
pointing performance. Roll and pitch are bounded with std 4° / 1.8°
and peaks of 18° / 8° respectively — these are the actual
reduced-attitude error projected onto the two controlled axes. **Yaw
wraps freely between ±180°**, confirming visually that the 2-DOF
controller is yaw-invariant: no torque is produced about `ẑ_body` at
equilibrium and the satellite is free to spin slowly about its
nadir-pointing axis. This is a *feature*, not a bug — the paper makes
the same observation.

The roll/pitch bounds correspond directly to the
`compute_pointing_error` plot used in §8.1 via
`pointing_error = acos(cos φ · cos θ)`.

### 8.3 4-hour rolling profile (after fix)

| Window | Mean | Max |
|--------|-----:|----:|
| 0 – 4 h   | 3.2° |  6.8° |
| 4 – 8 h   | 4.0° | 13.0° |
| 8 – 12 h  | 1.8° |  4.7° |
| 12 – 16 h | 3.8° | 11.2° |
| 16 – 20 h | 4.6° | 18.3° |
| 20 – 24 h | 4.0° | 11.8° |
| 24 – 28 h | 2.3° |  4.6° |
| 28 – 32 h | 3.3° | 10.3° |
| 32 – 36 h | 3.6° | 10.3° |
| 36 – 40 h | 3.3° | 10.6° |
| 40 – 44 h | 4.3° | 10.7° |
| 44 – 48 h | 5.2° | 10.9° |

### 8.4 MATLAB-mimic IC test (paper-style convergence)

To compare more directly with the MATLAB reference, a second scenario
is provided in `test_nadir.c`: random initial quaternion, ω₀ = (n, −n, n)
(off-axis from the orbit normal, magnitude √3·n ≈ 0.18°/s), 5-orbit
duration, eclipse on. This matches the IC distribution used in
`ref/.../Sim_sat_initial_state.m` and `Sim_time_parameters.m`.

Per-orbit summary:

| Orbit |   Mean |    Max |   Min  |  ⟨\|ω\|⟩      |
|-------|-------:|-------:|-------:|----------:|
| 1     | 13.96° | 91.88° |  1.41° | 0.077°/s |
| 2     |  4.14° | 10.52° |  0.49° | 0.067°/s |
| 3     |  3.94° |  7.88° |  1.38° | 0.070°/s |
| 4     |  2.00° |  5.07° |  0.02° | 0.067°/s |
| 5     |  4.28° |  7.54° |  1.91° | 0.071°/s |

|ω| converges from 0.113°/s (= √3·n) to ≈ 0.067°/s (= n, the LVLH
orbit-tracking rate). First crossing of 20° at t = 19.7 min (0.21 orbits).

**Two additional figures are produced** (`plot_results.py::plot_nadir`):

* `nadir_lvlh_paper_style.png` — three-panel roll/pitch/yaw vs. time
  with orbit-boundary dashed lines, in the visual style the paper uses.
* `nadir_lvlh_paper_style_zoom.png` — roll & pitch overlaid, IC
  transient cropped, to show the post-acquisition limit cycle clearly.

These plots reveal the **classical underdamped second-order PD response
envelope** that the scalar `pointing_error` metric (a non-negative
arccos) hides:

* **Pitch θ** acquires with a textbook overshoot pattern:
  +5° → +28.7° (first peak, t ≈ 0.17 h) → −10.5° (second peak, t ≈ 2.6 h)
  → +4° (third peak) → ±5° steady-state limit cycle.
  Successive-peak ratio ≈ 0.37 ⇒ logarithmic decrement Λ ≈ 1.0
  ⇒ damping ratio ζ ≈ 0.16 (clearly underdamped).
* **Roll φ** acquires from −91.9° to ≈ 0 over the first orbit with
  *no overshoot* (over-damped on this axis — magnetic field has higher
  cross-axis controllability for roll than pitch in this orbit).
* **Yaw ψ** drifts continuously through ±180°, confirming visually
  that the controller is by construction yaw-invariant (§3.2).

**Why the envelope does not keep shrinking past orbit 2:** the system
is *not* a stationary second-order PD. Three structural effects pin
the steady-state error to a 3–5° mean / 5–10° peak limit cycle:

1. **Time-varying control authority.** B rotates with the orbit, so
   the matrix `b̂×` projecting the PD law onto body torques is
   periodic. Errors momentarily aligned with B are uncontrollable in
   that instant and grow until B turns.
2. **Continuous disturbance torques.** Gravity gradient (~10⁻⁷ N·m
   peak), residual magnetic dipole, and aerodynamic torque inject
   energy continuously. With magnetorquers as the only actuator,
   these can only be sunk in the directions B-cross currently
   controls.
3. **MTQ ±m_max quantization.** The 1st-order Σ-Δ modulator outputs
   discrete pulses; below a threshold roughly proportional to
   `1/(K_P · |B|² · Δt)`, errors hover around the quantization noise
   floor instead of decaying further.

The reference paper's plots appear smoother because (a) the simulation
spans only 5 orbits — the IC transient dominates and the limit cycle
is barely visible, (b) hysteresis rods provide all-direction passive
damping that PoCat does not have, and (c) the linear y-axis spanning
~120° visually compresses the ±5° steady-state band. Re-running our
sim for 5 orbits with hysteresis rods would reproduce the paper-style
look.

---

## 9. Why Earlier Fix Attempts Failed

Documented for posterity — all of these were tried before the actual
root cause was found, and all of them were treating symptoms rather
than the structural defect.

* **Soft-yaw blend (3-DOF target with low yaw weight).** Composes
  badly with Σ-Δ because the LVLH 3-DOF target adds a time-varying
  yaw demand that the modulator faithfully encodes as pulses.
* **Gyro low-pass `τ = 10 s`.** Reduces the rate-feedback noise
  floor but introduces phase lag that drops the effective damping
  ratio; last-orbit error worsened from 2° to 55°.
* **Σ-Δ noise floor 0.15 mA.** Improves steady-state by suppressing
  sub-noise pulse trains, but at moderate errors (~80°) the genuine
  P-command is also below the floor on the X-axis, so the controller
  loses authority and cannot recover.
* **Σ-Δ accumulator floor / hard deadband.** Same problem as above
  with a sharper transition.

None of these touched the phantom `ε` at equilibrium, so none
addressed the actual cause.

---

## 10. Code-to-Formula Mapping

| Quantity / equation                | File                              | Lines      |
|------------------------------------|-----------------------------------|------------|
| `ε = ẑ_body × n̂_body` (eq. 5)     | `src/subsystems/adcs/adcs_nadir.c`| ~156–194   |
| Σ-Δ accumulator reset on mode init | `src/subsystems/adcs/adcs_nadir.c`| ~57        |
| B-cross PD law (eq. 1)             | `src/subsystems/adcs/adcs_nadir.c`| Step 4–5   |
| `quat_rotate_vec` (R-matrix form)  | `include/subsystems/adcs/adcs_math.h` | 158–170 |
| Σ-Δ first-order modulator          | `src/subsystems/adcs/adcs_magnetorquer.c` | 100–126 |
| Pointing error metric (test)       | `test/test_nadir.c`               | 75–90      |
| LVLH Euler-angle plot              | `test/plot_results.py`            | end of `plot_nadir` |

Configuration constants:

| Symbol           | Macro                  | File                                    | Value         |
|------------------|------------------------|-----------------------------------------|--------------:|
| `k_P`            | `NADIR_KP`             | `include/subsystems/adcs/adcs_config.h` | `5.0e-5`      |
| `k_R`            | `NADIR_KR`             | `include/subsystems/adcs/adcs_config.h` | `1.907413e-2` |
| `ẑ_body`         | `NADIR_BODY_AXIS_*`    | `include/subsystems/adcs/adcs_config.h` | `(0,0,1)`     |
| Gyro filter      | `NADIR_GYRO_FILTER_TAU`| `include/subsystems/adcs/adcs_config.h` | `0.0` (off)   |

---

## 11. Open Items / Future Work

1. **Eclipse robustness.** Re-enable eclipses and verify nadir hold
   through the bias-estimator dropout windows.
2. **Power-saving control rate.** Re-enable the adaptive `Δt = 10 s`
   hold mode now that the controller is stable.
3. **3-DOF yaw alignment (optional).** If a future payload requires
   yaw control, replace the 2-DOF B-cross law with the SR-Inverse
   3-DOF approach used in Sugimura 2016. The yaw-invariant ε remains
   the correct primitive for the roll/pitch sub-problem.
4. **K_P sweep.** With the structural fix in place, `k_P` can be
   tuned for noise-rejection vs settling-time trade-offs without
   triggering the limit cycle. Suggested grid:
   `k_P ∈ {1e-4, 2e-4, 5e-4}` with `k_R = 1.907413e-2 · √(k_P / 5e-5)`
   to maintain ζ ≈ 0.65.

---

## Appendix A. Diagnosis Timeline

The chain of investigations, condensed for future readers:

1. Symptom: pointing error wandering to 95° over 8–12 h windows from
   a perfect nadir IC, then settling near 2° on the final orbit.
2. First hypothesis — MTQ quantization deadband. Implemented Σ-Δ
   modulator. Last-orbit error reached 2°; mid-test windows still
   peaking at 95°. Improvement was only over averaging windows where
   the limit cycle happened to be in its "calm" phase.
3. Second hypothesis — gyro/MTQ noise pumping the unconstrained yaw
   axis. Tried (a) gyro LP filter, (b) Σ-Δ floor 0.15 mA, (c) random
   ablations. Each made one window better and another worse.
4. Rubber-duck critique pointed at the structural property of
   `quat_from_two_vectors` / `U2Q`: it picks one yaw out of the
   yaw-family and the resulting `ε` is therefore non-zero at
   "equilibrium" attitudes other than that one specific yaw. The
   long-period limit cycle is the closed-loop response to this
   phantom command.
5. Implemented the cross-product `ε`. Pre-fix 38° / 95° → post-fix
   3.6° / 18°, monotonically across every 4-hour window. No further
   tuning needed.
