//! FFI to the firmware's `curve_interp` library.
//!
//! `build.rs` compiles `components/curve_interp/curve_interp.c` into this crate,
//! so simulating a scene here runs the *exact* fixed-point code the ESP32 runs in
//! its ISR. The host simulation is therefore bit-exact with the device — it is
//! ground truth for the velocity profile, the emitted setpoints, and the limit
//! checks. See docs/CURVE_MOTION.md.

/// Galvo motion limits — mirrors `curve_limits_t` (counts / second / µs).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CurveLimits {
    pub v_max_cps: i64,
    pub a_max_cps2: i64,
    pub dt_tick_us: i32,
}

impl CurveLimits {
    /// The device's defaults, read straight from the firmware (`curve_interp.h`)
    /// over FFI — never mirrored or guessed here, so they can't drift.
    pub fn device_default() -> Self {
        // SAFETY: trivial value-returning C function, no arguments.
        unsafe { curve_default_limits() }
    }
}

impl Default for CurveLimits {
    fn default() -> Self {
        Self::device_default()
    }
}

/// Convert a planned speed (counts/second) to the wire/firmware format
/// (counts/tick * 256). Defers to the firmware's `curve_cps_to_wire` over FFI, so
/// the host and device share one definition of the crossing — used by `emit` (real
/// wire bytes) and `run_curve` (firmware simulation).
pub fn cps_to_wire(v_cps: f64, dt_us: i32) -> u32 {
    // SAFETY: pure arithmetic C function, no pointers.
    let w = unsafe { curve_cps_to_wire(v_cps.round() as i64, dt_us) };
    w.clamp(0, u32::MAX as i64) as u32
}

/// Mirrors `curve_state_t` exactly (field order + types) so the by-value
/// allocation size matches. Opaque to callers; we never read these directly —
/// `curve_speed_cps` does the counts/s readout. Groups match the C struct: shape,
/// tick-unit limits, evolving state.
#[repr(C)]
struct CurveState {
    // shape
    ax: i64,
    bx: i64,
    cx: i64,
    dx: i64,
    ay: i64,
    by: i64,
    cy: i64,
    dy: i64,
    s: i64,
    p3x: i32,
    p3y: i32,
    // limits (tick units)
    vshift: i32,
    v_max: i32,
    a_max: i32,
    v_out: i32,
    // evolving
    t: u64,
    s_done: i64,
    v: i32,
    done: bool,
}

extern "C" {
    fn curve_default_limits() -> CurveLimits;
    fn curve_interp_begin(
        st: *mut CurveState,
        lim: *const CurveLimits,
        p0x: i32,
        p0y: i32,
        p1x: i32,
        p1y: i32,
        p2x: i32,
        p2y: i32,
        p3x: i32,
        p3y: i32,
        v_in_wire: i64,
        v_out_wire: i64,
    );
    fn curve_interp_step(
        st: *mut CurveState,
        out_x: *mut u16,
        out_y: *mut u16,
        carry_v_wire: *mut i64,
    ) -> bool;
    fn curve_speed_cps(st: *const CurveState, dt_tick_us: i32) -> i64;
    fn curve_cps_to_wire(v_cps: i64, dt_tick_us: i32) -> i64;
}

/// One emitted galvo setpoint from the interpolator, plus the exact speed the
/// firmware is at when it lands.
#[derive(Clone, Copy, Debug)]
pub struct Setpoint {
    pub x: u16,
    pub y: u16,
    pub v_cps: i64,
}

/// Run one CURVE through the real interpolator, returning every setpoint it
/// emits (one per `dt_tick_us`). `v_in_cps`/`v_out_cps` are the planner's physical
/// speeds (counts/second); they cross to the firmware's wire/tick format here, the
/// same way `emit` does, so this is a bit-exact replay of the device. `carry`
/// receives the exit speed in wire units (counts/tick * 256).
pub fn run_curve(
    lim: &CurveLimits,
    p0: [i32; 2],
    p1: [i32; 2],
    p2: [i32; 2],
    p3: [i32; 2],
    v_in_cps: i64,
    v_out_cps: i64,
    carry: &mut i64,
) -> Vec<Setpoint> {
    let v_in_wire = cps_to_wire(v_in_cps as f64, lim.dt_tick_us) as i64;
    let v_out_wire = cps_to_wire(v_out_cps as f64, lim.dt_tick_us) as i64;
    // SAFETY: begin fully initializes st before any read; the loop only borrows
    // st mutably through the C calls and reads its fields between them.
    unsafe {
        let mut st: CurveState = std::mem::zeroed();
        curve_interp_begin(
            &mut st, lim, p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], p3[0], p3[1],
            v_in_wire, v_out_wire,
        );
        let mut out = Vec::new();
        loop {
            let (mut x, mut y, mut c) = (0u16, 0u16, 0i64);
            let going = curve_interp_step(&mut st, &mut x, &mut y, &mut c);
            // Read the speed back in counts/s for reporting (host-only helper).
            out.push(Setpoint { x, y, v_cps: curve_speed_cps(&st, lim.dt_tick_us) });
            *carry = c;
            if !going {
                break;
            }
            if out.len() > 4_000_000 {
                break; // safety: never spin forever
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fixed limits for tests — deliberately NOT the firmware defaults, so editing
    /// curve_interp.h to retune a real galvo never breaks the test suite.
    fn tlim() -> CurveLimits {
        CurveLimits {
            v_max_cps: 11_468_800,
            a_max_cps2: 57_344_000_000,
            dt_tick_us: 20,
        }
    }

    // The keystone: the C library links and runs, a straight curve at v_max holds
    // v_max and lands exactly on P3. (Mirrors the C gtest, proving the FFI path.)
    #[test]
    fn ffi_straight_reaches_endpoint() {
        let lim = tlim();
        let mut carry = 0;
        let pts = run_curve(
            &lim,
            [2768, 32768],
            [22768, 32768],
            [42768, 32768],
            [62768, 32768],
            lim.v_max_cps,
            lim.v_max_cps,
            &mut carry,
        );
        assert!(pts.len() > 10, "expected many setpoints, got {}", pts.len());
        let last = pts.last().unwrap();
        assert_eq!((last.x, last.y), (62768, 32768), "must snap to P3");
        for p in &pts {
            assert!(p.v_cps <= lim.v_max_cps, "never exceeds v_max");
            assert!(p.v_cps >= lim.v_max_cps * 9 / 10, "holds near v_max");
        }
    }
}
