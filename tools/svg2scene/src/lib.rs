//! svg2scene — convert SVG artwork into an **ILDA-style point stream** for the
//! shadowgraph projector.
//!
//! Pipeline: `parse` (usvg → cubics) → flatten + fit (`kurbo`, → normalised
//! polylines) → `optimize` (`lasy` euler-circuit draw-order + blanking + corner
//! dwell, interpolated to a dense point stream) → `emit` (device-native
//! `laser_point_t` blob and/or a standard `.ild` file). There is **no curve /
//! CNC motion planning** — the uniform dense point stream is the output, exactly
//! what the firmware's fixed-rate ILDA engine consumes.

pub mod emit;
pub mod model;
pub mod optimize;
pub mod parse;
pub mod stream;
pub mod viz;
