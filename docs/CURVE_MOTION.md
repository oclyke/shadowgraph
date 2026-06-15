# Curve motion: a CNC-style Bézier path engine for the galvos

**Status:** design / proposal. Nothing implemented yet. This document is the
contract that the host tool and the firmware both build against.

## 1. Goal

Drive the galvos from **cubic Bézier segments** carrying **entry/exit velocities**
instead of dense point streams. The host fits the artwork to curves and does the
*global* velocity planning (feasibility, look-ahead); the firmware does the
*local*, real-time job of riding each curve at the maximum speed the galvo
physics allow, sampling its own setpoints in the timer ISR.

Why:

- **Wire efficiency.** A straight edge or gentle arc is ~17 bytes (one `CURVE`),
  not hundreds of `GOTO`s. Great for the UDP streaming path.
- **Maximum frame rate.** The beam runs at the friction-circle limit everywhere,
  not at a host-guessed `max_step_us` pacing. Persistence-of-vision (frame time)
  falls out as a *reported byproduct*, never an input — same principle as before.
- **Smooth straights.** The firmware samples curves at its native ISR rate, so
  the old "instant `GOTO` → dotted straight" tradeoff disappears.

This is the standard **two-stage CNC architecture**: a *look-ahead / feedrate
planner* (host) feeds a *real-time interpolator* (firmware).

## 2. Architecture & division of labor

```
            HOST  (tools/svg2scene, Rust)                 FIRMWARE (ESP32, C)
  ┌──────────────────────────────────────────┐     ┌──────────────────────────┐
  │ parse SVG ─ keep cubics (usvg + kurbo)    │     │ scene_stream / UDP        │
  │ fit to field [-1,1], y-up                 │     │   → byte_queue            │
  │ order + blank (lasy euler circuit)        │     │                           │
  │ SEGMENT cubics at corners / inflections / │     │ laser_engine ISR          │
  │   curvature extrema (monotone-κ pieces)   │     │  pop CURVE                 │
  │ JUNCTION velocities (curvature / corner   │     │  curve_interp_begin()     │  ← shared
  │   deviation)                              │     │  per tick:                │     C lib
  │ GLOBAL look-ahead (fwd/back v² passes)    │     │   curve_interp_step()  →  │  ← curve_interp
  │ EMIT  CURVE{P1,P2,P3,v_in,v_out}  + LASER │ ──▶ │   GOTO(x,y); arm(Δτ)      │     (THE contract)
  │                                           │ wire│  on t≥1 pop next cmd      │
  │ SIMULATE via FFI to curve_interp (exact)  │◀────┼── identical algorithm ────┘
  │ analyze / viz / limit-validation tests    │     │ (same source file)        │
  └──────────────────────────────────────────┘     └──────────────────────────┘
```

The keystone is **`curve_interp`**: one small, dependency-free, fixed-point C
library that implements the per-tick interpolation. The *firmware ISR calls it*
and the *host links the very same file* (via `cc` in `build.rs`) to simulate
exactly what the device will draw. There is no second implementation to drift.

| Concern | Host (planner) | Firmware (interpolator) |
|---|---|---|
| Geometry source | SVG → cubics | receives cubics |
| Where to slow (curvature) | sets junction v + segments so κ is monotone | clamps to local κ ceiling per tick |
| Corners | junction-deviation v | obeys the v it's given |
| Global feasibility | fwd/back v² look-ahead over all segments | trusts v_in/v_out are reachable |
| Max-accel motion | guarantees a feasible envelope exists | rides the friction circle in real time |
| Frame time | reports Σdt from a simulation | produces the actual dt (ISR ticks) |

## 3. The motion model (the math)

### 3.1 Path ≠ time
A cubic `B(t), t∈[0,1]` is geometry only. Motion adds a time map `τ ↦ t(τ)` —
*how fast we sweep t*. We plan the **feedrate profile** `v(s)` = tangential speed
vs. arc length `s`, decoupled from the path. This is Time-Optimal Path
Parameterization (TOPP).

```
B(t)   = (1-t)³P₀ + 3(1-t)²t P₁ + 3(1-t)t² P₂ + t³ P₃
B'(t)  = tangent
ds     = |B'(t)| dt
κ(t)   = |B'(t) × B''(t)| / |B'(t)|³        (curvature; closed form)
```

### 3.2 Friction circle = "maximum acceleration"
Moving at speed `v`, acceleration splits into tangential and normal:

```
a_t = dv/dτ                 (speed change)
a_n = v²·κ                  (centripetal — the cost of turning)
|a| = √(a_t² + a_n²) ≤ a_max
```

So the usable tangential budget is what's left after paying for the turn:

```
a_t,avail = √( a_max² − (v²κ)² )         ← ride this = go for max acceleration
v_ceiling = min( v_max, √(a_max / κ) )   ← can't even hold the turn faster than this
```

### 3.3 Junction velocities (the v_in / v_out you asked about)
Each segment boundary gets one scalar speed (both segments share the point, so a
scalar is enough — direction change is absorbed by *how low* it is):

- **Tangent-continuous (C1) joint:** `v_j = min(v_max, √(a_max/κ_j))`.
- **Corner (tangent jump):** *junction deviation* (Grbl/Marlin). Inscribe an arc
  of radius `r` so it deviates from the vertex by ≤ `δ` (one physical knob, the
  galvo's allowed corner-rounding); `v_j = √(a_max·r)`,
  `r = δ·sin(θ/2)/(1−sin(θ/2))`, `θ` = turn angle. A spike → v_j→0; a gentle
  kink → barely slows. This **replaces the old pivot-to-v_min hack** with a
  smooth, accel-bounded corner speed.
- **Stroke start/end, blank transitions:** `v = 0`.

### 3.4 Global look-ahead (host)
With `a(s)=v²` the accel constraint is linear: `a_t = ½ da/ds`. So a forward then
backward pass over the *chain of junction velocities* makes every junction
mutually reachable:

```
forward:   v_out[k]² ≤ v_in[k]² + 2·a_max·S[k]
backward:  v_in[k]²  ≤ v_out[k]² + 2·a_max·S[k]
```

(`S[k]` = segment arc length.) After this pass each `CURVE` ships a `v_in/v_out`
pair that is *guaranteed feasible*. This is exactly the old forward/backward accel
passes — lifted from per-point to per-segment-endpoint.

### 3.5 Why the host splits cubics
If we split each cubic at **corners, inflection points (κ=0, analytic), and
curvature extrema**, then within every emitted piece curvature is *monotone* — so
the binding curvature limit always sits at a *junction*, captured by `v_in/v_out`.
That lets the firmware get away with **endpoint-only braking** (it never has to
look ahead for a curvature spike hidden mid-segment). Big firmware simplification.

## 4. Wire format

New TV record, additive — `GOTO`/`LASER`/`DWELL` stay exactly as they are, so old
scenes still play and we can A/B.

```
LASER_CMD_CURVE = 0x04
  P1.x  u16   P1.y  u16        control point 1   (LE, DAC counts, center 0x8000)
  P2.x  u16   P2.y  u16        control point 2
  P3.x  u16   P3.y  u16        end point (= next segment's P0)
  v_in  u32                    entry speed, counts/second
  v_out u32                    exit  speed, counts/second
                               total = 1 + 12 + 8 = 21 bytes
```

Decisions baked in:

- **P0 is implicit** = the engine's current position (end of the previous
  command). Saves 4 bytes *and* guarantees C0 continuity by construction.
- **Color is not in `CURVE`.** A preceding `LASER` sets it and it's held — exactly
  like `GOTO` today. A **blank travel move** is `LASER 0,0,0` then a (degenerate,
  straight) `CURVE`. The interpolator is identical for lit and blank moves.
- **Wire velocity unit = counts/second, `u32`.** Bandwidth is not a constraint in
  this scheme, so we keep the wire *human-readable* (you literally see
  `11500000`) with no Q-format to remember; range to 4.3 G, resolution 1 count/s.
  This is intentionally decoupled from the firmware's *internal* representation
  (below) — the firmware converts once per command.
  - *Why not counts/µs or counts/ns on the wire?* As plain integers they'd be
    useless: v_max ≈ 11.5 counts/µs ≈ 0.0115 counts/ns → ~12 distinct values, or
    all zeros. Fine time units only help with fractional (Q-format) bits, which is
    what the internal representation uses; the wire stays integer counts/s.
- **Internal representation = large fixed-point.** The interpolator computes in
  `Q16.16` over counts & µs with **64-bit intermediates** (the centripetal `v²`
  term reaches ~1e14, so 64-bit products are required regardless). Exact formats
  are finalized in the `curve_interp` implementation with documented overflow
  headroom.
- **Limits (`v_max`, `a_max`, `δ`, `Δτ`) are NOT on the wire.** They are galvo
  properties living in a shared config header (§5.3); the host CLI mirrors them
  and a test asserts host/firmware agree. (Future: a `CONFIG` command to push
  them at runtime.)

## 5. The shared interpolator — `components/curve_interp`

A new component: pure C, no ESP-IDF headers, **fixed-point**, host-testable. This
is the single source of truth for "what does a `CURVE` actually draw."

### 5.1 Public API (sketch)
```c
typedef struct {            // immutable galvo limits (shared config, §5.3)
    int32_t v_max;          // counts/s
    int32_t a_max;          // counts/s²
    int32_t dt_tick_us;     // interpolation period Δτ
} curve_limits_t;

typedef struct { /* fixed-point coeffs, arc length S, s_done, t, v, ... */ } curve_state_t;

// Once per CURVE: expand control points to Horner coeffs, integrate arc length S.
void curve_interp_begin(curve_state_t *st, const curve_limits_t *lim,
                        int32_t p0x,int32_t p0y, int32_t p1x,int32_t p1y,
                        int32_t p2x,int32_t p2y, int32_t p3x,int32_t p3y,
                        int32_t v_in_cps, int32_t v_out_cps);   // wire unit: counts/s

// One ISR tick: advance Δτ, output next setpoint. Returns false when t≥1 (done);
// *carry_v receives the speed to hand to the next segment.
bool curve_interp_step(curve_state_t *st, uint16_t *out_x, uint16_t *out_y,
                       int32_t *carry_v);
```

### 5.2 The per-tick algorithm
Per `curve_interp_begin`: build cubic Horner coefficients from `P0..P3`; estimate
arc length `S` with one N-sample pass (`Σ|B'(tᵢ)|·Δt`, N≈16). No per-tick table
needed — monotone-κ pieces (§3.5) mean only endpoint braking.

Per `curve_interp_step` (fixed-point):
```
s_rem   = S - s_done
v_brake = sqrt(v_out² + 2·a_t·s_rem)          // decelerate in time to hit v_out
v_curv  = sqrt(a_max / κ(t))                   // local curvature ceiling
v_acc   = sqrt(v² + 2·a_t·Δs)                  // a_t = sqrt(a_max² − (v²κ)²)
v       = min(v_max, v_acc, v_brake, v_curv)
Δs      = v · Δτ
Δt      = Δs / |B'(t)|                          // 1st-order Taylor advance (CNC)
t      += Δt ; s_done += Δs
p       = B(t)                                  // Horner eval, ~3 mul + 3 add /axis
emit GOTO(p) ; arm timer for Δτ
if t ≥ 1: snap to P3, return done, carry final v
```

Notes:
- **Arbitrary-t eval (Horner), not fixed-step forward differencing** — adaptive
  speed means a *fixed time* step with a *variable arc* step, which needs B(t) at
  arbitrary t. (Forward differencing is only for fixed parameter steps.)
- 1st-order Taylor causes mild feedrate ripple; a 2nd-order term or a one-step
  Newton on `t(s)` removes it — Phase 2 polish if scope/ripple warrants.
- A few 32×32→64 mults and ~3 integer sqrts per tick; trivial at 240 MHz / ~10 µs.

### 5.3 Sharing & config
- `curve_interp.c/.h` compiles into firmware (called from the ISR) **and** into
  the host tool via `build.rs` (`cc` crate) for bit-exact simulation. Same file →
  no drift, by construction.
- `curve_limits_t` defaults live in `curve_interp.h`; host CLI flags mirror them;
  a unit test asserts the two are identical.
- **Fixed-point, not float, because the consumer runs in the gptimer ISR** and
  the FPU isn't saved on ISR entry under the current IDF config. (Alternative,
  flagged in §10: move interpolation to a pinned high-priority task that may use
  float — but that reverses the recent "consumer in the ISR" decision.)

## 6. Firmware integration — `laser_engine`

The engine gains a **curve mode** in its ISR state machine. Today `drain()`
processes instantaneous commands and arms on `DWELL`. New behavior:

- Pop `CURVE` → `curve_interp_begin(...)`, enter curve mode.
- Each subsequent ISR fire while in curve mode → `curve_interp_step(...)`, write
  the galvo DACs (reuse the existing `dispatch`/`galvo_write` path), re-arm for
  `Δτ` via the existing anchored-deadline machinery.
- `curve_interp_step` returns done → leave curve mode, carry `v` into the next
  command (a following `CURVE`'s `v_in` should match; assert in the host sim),
  continue draining.
- Underrun / corruption handling unchanged (blank + retry). `P0` for a curve =
  last written galvo position, tracked in engine state.

`laser_command` gains `LASER_CMD_CURVE`, `laser_command_push_curve(...)`,
`laser_command_pop` decode, and `laser_command_size` entry. `laser_engine` gains
`laser_engine_curve(...)` producer wrapper.

## 7. Host pipeline — `tools/svg2scene` (Rust, rebuilt)

Stages (each independent + testable, with a debug dump — the prior project value):

1. **parse** (`usvg` + `kurbo`): keep **cubics** (do *not* flatten). Lines = trivial
   cubics. Per-path color = stroke→fill→white. → `Vec<Subpath{color, Vec<CubicBez>}>`.
2. **fit**: normalize bbox → field `[-1,1]`, y-up flip.
3. **order + blank** (`lasy`): reuse the euler-circuit for draw order + minimal
   blanking **only**. lasy's data model is points + straight segments — it never
   sees curves. Feed it a *throwaway, coarse* polyline (each subpath's cubic
   **knot** points) purely to decide (a) draw order across subpaths and (b) where
   the minimal blank jumps go; then map that order/orientation back onto the
   **original cubics**, which are emitted untouched. Flattening is demoted to a
   routing-only scratch computation, so smoothness is never lost (this is the fix
   to the old tool, where the whole pipeline was point-based and flattened up
   front). lasy's interpolation (its point-stuffing) is dropped entirely. A
   connected curve subpath stays intact (possibly reversed); lasy only makes
   choices at genuine shared vertices (e.g. a figure-8 crossing), where a real
   corner exists anyway — and its angle-minimization there picks the least-turning
   continuation, which feeds straight into junction-deviation speeds.
4. **segment** (`kurbo` `curvature`, inflections, `arclen`): split each cubic at
   corners, inflection points, curvature extrema → monotone-κ pieces; cap max arc
   length. → ordered list of well-behaved segments + junction types.
5. **plan**: junction velocities (§3.3) → global fwd/back look-ahead (§3.4) →
   feasible `v_in/v_out` per segment.
6. **emit**: per segment, `LASER` if color changed, then `CURVE{P1,P2,P3,v_in,v_out}`;
   blank travels as `LASER 0` + straight `CURVE`. Quantize to u16 / counts-per-ms.
7. **simulate + analyze** (FFI → `curve_interp`): replay the exact firmware
   algorithm to get real setpoints; reconstruct windowed v/a/j; assert ≤ limits;
   report Σdt / refresh.
8. **viz**: `parse.svg`, `optimize.svg` (path colored by speed/accel), `curves.svg`
   (control points + per-segment v_in/v_out), `points.svg` (simulated setpoints),
   `profile.svg` (v(s)/a(s) vs limits), `scene.bin`. Keep the Bazel-style
   `--debug-output-dir` bundle.

## 8. Testing & validation

- **`curve_interp` host gtest** (matches existing `host_test` pattern):
  straight cubic + v_in=v_out → constant speed, exact arc length; high-κ arc with
  low v → centripetal ≤ a_max; v_in=0 ramp → a_t ≤ a_max and hits v_out;
  reconstructed (windowed) a(t), j(t) ≤ limits; Σdt matches analytic traversal.
- **Cross-check**: host Rust sim (FFI) vs firmware = identical *by construction*
  (same TU); a golden test pins a few scenes' setpoint streams.
- **`laser_command` gtest**: `CURVE` round-trip, size, FIFO order (extend existing
  file).
- **Rust pipeline tests**: junction feasibility after look-ahead; emitted wire
  decodes; limits respected on fixtures (triangle, circle, S-curve).
- **On-device**: a demo scene (single curve, chained curves, a glyph); scope the
  galvo / logic-analyze SPI to confirm smoothness and frame time.

## 9. Phasing

- **Phase 0 — Contract (keystone). ✅ DONE (2026-06-15).** `LASER_CMD_CURVE` in
  `laser_command` (21-byte record, +gtest, 8/8); `components/curve_interp` lib +
  host gtest (6/6, fixed-point, the per-tick algorithm, shared config header).
  The algorithm is proven on host and the byte format is fixed. As-built notes
  from the implementation:
  - **Asymmetric rate clamp.** Acceleration is limited to the friction-circle
    budget `a_t` per tick; braking is allowed up to full `a_max`. (Braking always
    reduces the centripetal load, and the v_out brake / curvature recovery need
    the full budget.)
  - **Curvature ceiling is a HARD clamp**, not rate-limited — `v` may never exceed
    `√(a_max·R)` (that would be >a_max centripetal). The host won't feed an
    infeasible `v_in`, but the firmware clamps hard if it ever sees one.
  - **Braking starts one nominal step early** (`s_rem − v·Δτ`) so discrete
    sampling never steps over the ideal brake point and overshoots `v_out`.
  - **Radius via `R = |B'|³/|B'×B''|`**, clamped to `R_MAX` (1e6 counts; above
    `v_max²/a_max ≈ 2300` curvature can't limit below v_max). `|B'|` capped at 2e6
    before cubing to stay inside int64.
  - A small **speed floor** (~1 count/tick) guarantees forward progress so every
    segment terminates. Each int64 product is annotated with its worst-case
    magnitude in `curve_interp.c` for overflow auditability.
- **Phase 1 — Firmware.** Curve-mode state machine in `laser_engine`; on-device
  demo. *Deliverable: device draws a hand-authored chained-curve scene.*
- **Phase 2 — Host tool.** Rebuild `tools/svg2scene` stages 1–8; FFI simulation &
  viz; limit tests. *Deliverable: SVG → scene.bin that the device draws.*
- **Phase 3 — Streaming & polish.** Port the UDP sender; animation (per-frame
  re-plan); 2nd-order interpolation / feedrate-ripple fix; jerk-limited (S-curve)
  envelope; tune limits to the real galvo datasheet.

## 10. Decisions (resolved 2026-06-15)

1. **ISR fixed-point — RESOLVED: fixed-point in the ISR.** Preserves the recent
   "consumer in the ISR" architecture (the FPU isn't saved on ISR entry). The
   same fixed-point `curve_interp` TU is linked into the host tool over FFI, so
   the host simulation is bit-exact with the device.
2. **Wire velocity unit — RESOLVED: `u32` counts/second.** Bandwidth is no longer
   a concern, so the wire is human-readable integer counts/s; the firmware uses a
   large internal fixed-point (`Q16.16`, 64-bit intermediates), decoupled from the
   wire. (Q8.8 / counts-per-µs-as-integer rejected — too coarse / all zeros.)
3. **Firmware autonomy — RESOLVED: one segment at a time.** The host's global
   forward/backward `v²` pass pre-solves feasibility, so the firmware only needs
   the single scalar `v_out` per segment (on the wire) and brakes locally with
   zero look-ahead. A Grbl-style device-side look-ahead buffer is unnecessary here
   (it exists only because Grbl's host ships no junction velocities) and would put
   the junction pass + a segment ring buffer in the time-critical path. The
   firmware carries its actual exit speed into the next curve's start; the host
   asserts `v_in[k+1] == v_out[k]`.
4. **Keep `lasy` — RESOLVED: yes, for ordering + blanking only**, run on a coarse
   knot polyline, original cubics carried through untouched (see §7 step 3).
   Curve-aware ordering (true Bézier tangents in the angle cost) is a later option.

## 11. References
- Bobrow, Dubowsky, Gibson (1985) — time-optimal path parameterization (phase plane).
- Pham (2018) — TOPP-RA (modern, robust). Verscheure et al. (2009) — convex/SOCP.
- Sonny Jeon — Grbl *junction deviation* cornering model.
- Yang & Kong; Bedi et al. — real-time parametric (NURBS/Bézier) CNC interpolation,
  Taylor feedrate, ripple compensation.
- Friction/traction circle — vehicle dynamics & robot path tracking.
- `kurbo`: `CubicBez`, `ParamCurveDeriv`, `ParamCurveCurvature`, `ParamCurveArclen`,
  `inv_arclen`, `fit_to_bezpath`.
