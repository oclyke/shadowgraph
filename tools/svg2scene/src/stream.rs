//! Push a scene to the device over TCP and wait for its commit ACK.
//!
//! Wire: `["SCN1"][u32 count LE][count × 8-byte laser_point_t records]`. The
//! device receives the points into its DRAM scene buffer, publishes them as the
//! active scene, and replies with a single ACK byte. Mirrors the old
//! `frame_send` ACK discipline so a follow-up can't race an in-flight scene.

use std::io::{Read, Write};
use std::net::TcpStream;

use crate::emit::encode_blob;
use crate::model::IldaPoint;

pub const DEFAULT_PORT: u16 = 7777;
pub const SCENE_MAGIC: &[u8; 4] = b"SCN1";

/// Send `scene` to `host` (`"ip"` or `"ip:port"`); blocks until the 1-byte ACK.
pub fn send_scene(host: &str, scene: &[IldaPoint]) -> std::io::Result<()> {
    let addr = if host.contains(':') {
        host.to_string()
    } else {
        format!("{host}:{DEFAULT_PORT}")
    };
    let mut stream = TcpStream::connect(&addr)?;
    stream.set_nodelay(true).ok();

    let mut hdr = Vec::with_capacity(8);
    hdr.extend_from_slice(SCENE_MAGIC);
    hdr.extend_from_slice(&(scene.len() as u32).to_le_bytes());
    stream.write_all(&hdr)?;
    stream.write_all(&encode_blob(scene))?;
    stream.flush()?;

    let mut ack = [0u8; 1];
    stream.read_exact(&mut ack)?;
    Ok(())
}
