//! Replay a planned scene through the real firmware interpolator (FFI) and
//! analyse the result. Because `interp` links the device's `curve_interp.c`, the
//! setpoints and speeds here are exactly what the ESP32 will produce.

use crate::interp::{self, CurveLimits};
use crate::model::{Move, Rgb};

/// One emitted galvo setpoint (one ISR tick), with the exact speed there.
#[derive(Clone, Copy)]
pub struct SimPoint {
    pub pos: [f64; 2],
    pub color: Rgb,
    pub blank: bool,
    pub v_cps: i64,
}

fn ci(x: f64) -> i32 {
    x.round().clamp(0.0, 65535.0) as i32
}

/// Simulate the whole move list; one CURVE at a time, carrying the exit speed.
pub fn simulate(moves: &[Move], lim: &CurveLimits) -> Vec<SimPoint> {
    let mut pts = Vec::new();
    let mut carry = 0i64;
    for m in moves {
        let c = &m.cubic;
        let sps = interp::run_curve(
            lim,
            [ci(c.p0.x), ci(c.p0.y)],
            [ci(c.p1.x), ci(c.p1.y)],
            [ci(c.p2.x), ci(c.p2.y)],
            [ci(c.p3.x), ci(c.p3.y)],
            m.v_in.max(0.0) as i64,
            m.v_out.max(0.0) as i64,
            &mut carry,
        );
        for s in sps {
            pts.push(SimPoint {
                pos: [s.x as f64, s.y as f64],
                color: m.color,
                blank: m.blank,
                v_cps: s.v_cps,
            });
        }
    }
    pts
}

pub struct Stats {
    pub points: usize,
    pub blanks: usize,
    pub frame_s: f64,
    pub max_v: f64,
    pub max_a: f64,
    pub max_j: f64,
}

impl Stats {
    pub fn refresh_hz(&self) -> f64 {
        if self.frame_s > 0.0 {
            1.0 / self.frame_s
        } else {
            0.0
        }
    }
}

/// Frame time (= point count × tick) and windowed peak kinematics. Speed is
/// exact (read from the interpolator); accel/jerk are reconstructed on an ~80 µs
/// window so the 1 µs dt quantisation doesn't fabricate noise.
pub fn stats(pts: &[SimPoint], lim: &CurveLimits) -> Stats {
    let dt = lim.dt_tick_us as f64 * 1e-6;
    let n = pts.len();
    let blanks = pts.iter().filter(|p| p.blank).count();
    let frame_s = n as f64 * dt;
    let max_v = pts.iter().map(|p| p.v_cps as f64).fold(0.0, f64::max);

    let w = ((80e-6 / dt).round() as usize).max(1);
    let v = |i: usize| pts[i].v_cps as f64;
    let mut accel = vec![0.0f64; n];
    for i in 0..n {
        if i >= w && i + w < n {
            accel[i] = (v(i + w) - v(i - w)) / (2.0 * w as f64 * dt);
        }
    }
    let max_a = accel.iter().fold(0.0, |m, a| f64::max(m, a.abs()));
    let mut max_j = 0.0;
    for i in 0..n {
        if i >= w && i + w < n {
            let j = (accel[i + w] - accel[i - w]) / (2.0 * w as f64 * dt);
            max_j = f64::max(max_j, j.abs());
        }
    }

    Stats {
        points: n,
        blanks,
        frame_s,
        max_v,
        max_a,
        max_j,
    }
}
