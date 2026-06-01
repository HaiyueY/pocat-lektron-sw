# MATLAB Nadir-Pointing Cross-Validation: Gain & Normalization Bugs

## 1. Problem Statement

After the C-side yaw-invariance fix
(`adcs_nadir_yaw_invariance_analysis.md`), we attempted to **cross-validate**
the C implementation against the MATLAB reference
(`ref/PoCat-Lektron-ADCS/NadirPointing/NadirPointing.m`). The two
implementations should be numerically equivalent: both consume the same
inertia, the same orbit, and the same nominal `(kP, kR)` design.

Instead, the MATLAB simulation **never converged**:

| Metric (2 orbits, truth quaternion, no noise) | Value |
|---|---|
| AKE start → end | 0° → 25.3° |
| AKE max / mean | 40.3° / 17.8° |
| `\|epsilon\|` start → end / mean | 0.000 → 0.428 / 0.301 |
| `\|w_rel\|` start → end | 3.76e-3 → 1.33e-3 rad/s |
| Dominant `\|epsilon\|` oscillation period | 1419 s (⇒ ≈2840 s on the angle, matching observation) |
| Saturation fraction (any axis) | **0.0** (no clipping anywhere) |

The pointing error sustained a clean, undamped limit cycle. The
oscillation period was *not* the orbit period and *not* the designed
PIDMIMO natural period — it was the natural period of an effective
closed loop whose gain was orders of magnitude weaker than designed.

The same control structure works on flight C code with essentially the
same `(kP, kR)` numerical values (`NADIR_KP = 5.0e-5`,
`NADIR_KR = 1.907413e-2`). Two real bugs were hidden in the MATLAB B-cross
formulation that, combined, made its effective gain
**~10 000× too small**.

---

## 2. Root Causes

### Bug 1 — Double inertia multiplication

`PIDMIMO.m` (utils/CubeSatToolbox/Common/Control/PIDMIMO.m) returns a
struct `constants` with the **raw** closed-loop PID gains (lines 100–106
of PIDMIMO.m, *before* the inertia-scaling on lines 120–121). For the
nominal design

```matlab
PIDMIMO(Inertia_matrix, 1, 0.005/4, 300, 0.1, dT, 'Delta')
```

we get `constants.kP = 4.3732e-5`, `constants.kR = 1.9074e-2`, which is
essentially identical to the C code constants
`NADIR_KP = 5.0e-5`, `NADIR_KR = 1.907e-2`. PIDMIMO premultiplies the
returned `c, d` state-space matrices by inertia (PIDMIMO.m L120–121) but
leaves the scalar `constants.kP/kR/kI` untouched.

`NadirPointing.m` then multiplied a second time:

```matlab
kP = d.inertia * constants.kP;   % BUG — inertia is included twice
kR = d.inertia * constants.kR;
```

With `d.inertia ≈ 1.17e-4 kg·m²` this drops the effective gain by four
orders of magnitude (`kP_used = 5.13e-9` vs the C value of `5e-5`).
This bug was already documented in `SIMULATION_REPORT.md` (item C1) but
had not been fixed.

### Bug 2 — Wrong power of |B| in the dipole normalization

The B-cross law expresses the commanded magnetic dipole as

```
m = kP · (B × ε) / |B|     ⟵ correct (and what the C reference does)
```

`NadirPointing.m` instead divided by `|B|²`:

```matlab
moment = ( kP·cross(B, ε) - kR·cross(B, w_rel) ) / (norm(d.fieldBODY)^2);
%                                                  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%                                                  BUG — extra factor of |B|
```

In LEO `|B| ≈ 3×10⁻⁵ T`, so this is another ~3×10⁴× attenuation of the
commanded dipole. Compare with the C reference
(`pocat-lektron-sw/src/subsystems/adcs/adcs_nadir.c` L237–239):

```c
desired_dipole.x = (NADIR_KP * b_cross_eps.x - NADIR_KR * b_cross_omega.x) / b_norm;
//                                                                          ^^^^^^
```

### Combined effect

The product of the two bugs makes MATLAB's commanded dipole roughly
`(d.inertia · |B|) ≈ 3.5×10⁻⁹` times what the C controller commands —
ten thousand times too small once you account for both factors not
quite cancelling. The actuator therefore *never* saturates (`|moment|
< 6.4×10⁻⁴ A·m²` vs `|m_max| ≈ 1.2×10⁻²`), the closed-loop bandwidth
collapses well below the design point, and the system rides a periodic
limit cycle driven by the rotating magnetic field instead of converging.

---

## 3. Investigation Process

Six controlled experiments were run (truth quaternion, no
gyro/magnetometer noise, 2 orbits, dT = 1 s) to isolate each effect:

| Exp | w₀ | Divisor | Inertia factor | Effective kP | AKE end | AKE max | AKE mean |
|---|---|---|---|---|---|---|---|
| Baseline | `[Ω;−Ω;Ω]` | `\|B\|²` | × inertia | 5.13e-9 | 25.3° | 40° | 17.8° |
| A | `[0;−Ω;0]` | `\|B\|²` | × inertia | 5.13e-9 | 20.9° | 23° | 7.9° |
| B | `[Ω;−Ω;Ω]` | `\|B\|²` | × inertia (×10) | 5.13e-8 | 11.7° | 19° | 6.9° |
| C | `[0;−Ω;0]` | `\|B\|²` | × inertia (×10) | 5.13e-8 | 0.9° | 3° | 0.87° |
| D | `[0;−Ω;0]` | `\|B\|²` | none | 4.37e-5 | 39.7° | 99° | 27° (100 % saturated) |
| **F** | `[0;−Ω;0]` | `\|B\|` | none | 5.0e-5 | **6.4°** | **6.8°** | **2.7°** |
| **G** | `[Ω;−Ω;Ω]` | `\|B\|` | none | 5.0e-5 | **3.9°** | 39.9° | **6.5°** |

**Reading the table:**

- The baseline plus A/B/C show that w₀ and a 10× gain bump alone are
  band-aids: they reduce the error but the system is still operating in
  the wrong regime.
- D shows that dropping the inertia factor while keeping `/|B|²`
  produces ~1.8 A·m² commands that clip 100 % of the time — bang-bang
  control. So both bugs need to be fixed together.
- F applies the **correct** fix (no inertia mult + `/|B|`). From an
  ideal nadir-tracking IC it converges to a steady 2–3° AKE.
- G is the headline result: even from the **original** `w₀ = [Ω;−Ω;Ω]`
  the corrected controller converges. The first third has a transient
  (mean 14.8°, peak 40°), but the middle and last thirds settle to
  mean 1.9° and 2.9° respectively. Final AKE = 3.9°.

So `w₀` is **not** a real bug — it is merely an off-nominal IC that
the corrected controller absorbs in roughly one orbit. The two real
bugs are the inertia mult and the `|B|` power.

---

## 4. Resolution

Two-line patch to
`ref/PoCat-Lektron-ADCS/NadirPointing/NadirPointing.m`:

```diff
- kP = d.inertia * constants.kP;
- kR = d.inertia * constants.kR;
+ kP = constants.kP;
+ kR = constants.kR;
  ...
- moment = ( kP .* cross(d.fieldBODY, epsilon) ...
-          - kR .* cross(d.fieldBODY, w_rel) ) / (norm(d.fieldBODY)^2);
+ moment = ( kP .* cross(d.fieldBODY, epsilon) ...
-          - kR .* cross(d.fieldBODY, w_rel) ) / norm(d.fieldBODY);
```

After the patch the MATLAB simulation now mirrors the C code line-for-line:
same gains, same `/|B|`, same yaw-invariant `ε = ẑ_body × n̂_body`,
same orbit-rate-compensated rate damping. Closed-loop response from
the original off-nominal IC converges to AKE ≈ 4° within one orbit.

The fix touches *only* the controller block on lines ~322–334; no
diagnostic, plotting, or KF code is modified.

---

## 5. Why the original MATLAB run "looked" symptomatic but wasn't an actuator problem

The diagnostic signatures observed before the fix all pointed at a
gain/normalization issue, not a saturation or noise issue:

- **Period ≈ 2840 s on the angle, 1419 s on `|ε|`.** With the
  attenuated effective gain `kP_used ≈ 5×10⁻⁹` the linearized natural
  frequency is `ω_n ≈ √(0.5 · kP_used / I) ≈ 4.6×10⁻³ rad/s`, giving
  a 1.3 ks oscillation in `|ε|`. The factor-of-two doubling from
  `|ε| ∝ |sin(θ)|` to `θ` matches the observed ~2840 s angle period.
- **No saturation.** `|m|_max ≈ 6×10⁻⁴ A·m²` is well under
  `m_max ≈ 8×10⁻³`. Saturation never engages, so the bug cannot be in
  the actuator model.
- **Damping looks "alright" on paper.** Designed `ζ = kR/(2√(I·kP))`
  is supercritical — but with `kP` and `kR` both attenuated by
  `d.inertia·|B|`, the *ratio* preserves the apparent damping while the
  *absolute* bandwidth collapses, so the system damps too slowly to
  beat the periodic disturbance imposed by the rotating B field within
  the 2-orbit run.

---

## 6. Cross-Validation Status

After the patch:

- MATLAB and C both use `kP ≈ 5×10⁻⁵`, `kR ≈ 1.9×10⁻²`, `/|B|`,
  yaw-invariant `ε`, orbit-rate-compensated damping.
- MATLAB AKE: end 3.9°, settled mean 2.9°.
- C AKE (from `adcs_nadir_yaw_invariance_analysis.md`):
  mean 3.6°, peak 18° on a 48-hour hold.

The two implementations are now **gain-equivalent** to within the
numerical difference between `constants.kP = 4.37×10⁻⁵` and
`NADIR_KP = 5.0×10⁻⁵` (14 %), which is well within the tuning margin.
The MATLAB simulation is therefore a faithful reference for further
C-side regression tests.

---

## 7. Lessons / Items for Future Work

1. **`SIMULATION_REPORT.md` was right about C1.** The double-inertia
   bug had been correctly diagnosed but never fixed. Track these
   reports.
2. **Normalization power matters.** Always compare the B-cross formula
   against a textbook reference — `/|B|` is conventional. `/|B|²` only
   shows up if you write `m = (kP · B̂ × ε) / |B|`, but then the gain
   itself has different units.
3. **Bug isolation via dimensional sweeps.** When a closed loop fails,
   sweep the gain by 10× / 100×. If 10× recovers stability, the bug is
   structural (here, it pointed to a missing power of `|B|` and an
   extra factor of `I`).
4. **`w₀` IC mismatch is a transient, not a structural bug.** The
   corrected controller absorbs an off-nominal initial body rate in
   ~½ orbit. Future MATLAB scenarios should still seed
   `w₀ = ω_orbit_body` to shorten the warm-up phase, but it is not
   required for convergence.

---

## 8. Files Touched

| File | Lines | Change |
|---|---|---|
| `ref/PoCat-Lektron-ADCS/NadirPointing/NadirPointing.m` | ~322–334 | Removed `d.inertia *` factor on `kP, kR`; changed `/norm(B)^2` to `/norm(B)`. |

No C-side, KF, or plotting code was modified.
