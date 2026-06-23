//! Shared data types flowing through the pipeline.

/// Linear RGB in `[0,1]` per channel.
pub type Rgb = [f32; 3];

/// A flattened, fitted pen-down run of one colour. Positions are **normalised to
/// `[-1,1]`, y-up** (the space `lasy` reasons about); `optimize` consumes these.
#[derive(Clone, Debug)]
pub struct Polyline {
    pub color: Rgb,
    pub pts: Vec<[f32; 2]>,
}

/// One emitted ILDA point — the host mirror of the firmware `laser_point_t`
/// (`components/point_ring`): signed 16-bit position centred at 0 (+Y up), 8-bit
/// colour, and a blank flag. `emit` serialises this to the device blob and `.ild`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IldaPoint {
    pub x: i16,
    pub y: i16,
    pub blank: bool,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

/// Status-byte flags — bit values identical to the firmware (`point_ring.h`) and
/// to the ILDA status byte, so the same bits serve the blob and the `.ild`.
pub const POINT_BLANK: u8 = 0x40; // beam off at this point
pub const POINT_LAST: u8 = 0x80; // last point of the frame
