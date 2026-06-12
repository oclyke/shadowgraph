//! `svg2scene`: convert SVG artwork into a galvo-ready laser command stream.
//!
//! The conversion is a pipeline of independent, individually testable stages —
//! see [`model`] for the data that flows between them and each stage module for
//! details:
//!
//! 1. [`parse`]    — SVG -> flattened coloured polylines (+ [`parse::fit_to_unit`]).
//! 2. [`optimize`] — `lasy` galvo optimisation (order, blanking, corner delays).
//! 3. [`emit`]     — points -> `GOTO`/`LASER`/`DWELL` commands and wire bytes.
//!
//! [`viz`] can dump any stage to an SVG for inspection.

pub mod analyze;
pub mod emit;
pub mod model;
pub mod optimize;
pub mod parse;
pub mod viz;
