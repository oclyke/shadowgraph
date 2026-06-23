//! svg2scene — convert SVG artwork into a single **standard ILDA frame** (`.ild`,
//! format 5) for the shadowgraph projector.
//!
//! Pipeline: `parse` (usvg → cubics) → flatten + fit (`kurbo`, → normalised
//! polylines) → `optimize` (`lasy` euler-circuit draw-order + blanking + corner
//! dwell, interpolated to a dense point stream) → `emit` (ILDA file). There is
//! **no curve / CNC motion planning** — the uniform dense point stream is the
//! output, exactly what the firmware's fixed-rate ILDA engine consumes. Playout
//! (streaming frames at a rate) is the separate `ildaplay` tool.

pub mod emit;
pub mod model;
pub mod optimize;
pub mod parse;
pub mod viz;
