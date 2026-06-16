//! svg2scene — convert SVG artwork into a cubic-Bézier (`CURVE`) galvo laser
//! command stream for the shadowgraph projector.
//!
//! Host-side planner half of the two-stage CNC architecture in
//! `docs/CURVE_MOTION.md`: parse → fit → order/blank → segment → plan (junction
//! velocities + global look-ahead) → emit CURVE; with a bit-exact simulation via
//! FFI to the firmware's `curve_interp`.

pub mod analyze;
pub mod emit;
pub mod interp;
pub mod model;
pub mod order;
pub mod parse;
pub mod plan;
pub mod segment;
pub mod viz;
