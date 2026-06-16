//! frame_send — the host side of the shadowgraph frame-streaming demo.
//!
//! Builds ONE closed-loop frame (a blob of laser_command bytes) and pushes it to
//! the device over TCP (the frame data plane). It then **waits for the device's
//! commit ack** on the TCP connection before sending a NEXT tick over UDP (the
//! playout clock), so the advance can't race ahead of the frame's bytes. Run it
//! repeatedly to stream an animation: each push becomes the displayed frame.
//! Playout is strictly FIFO (commit order) — frame ids are not used to address
//! frames. See docs/FRAME_STREAMING.md for the wire contract this matches.

use std::io::{Read, Write};
use std::net::{TcpStream, UdpSocket};
use std::path::PathBuf;

use clap::{Parser, ValueEnum};

// --- Wire constants (must match components/frame_stream/frame_stream.c) -------
const FRAME_MAGIC: u16 = 0x4652; // 'F','R'
const FRAME_ACK: u8 = 0x06; // device's 1-byte commit ack
const CTRL_MAGIC: u16 = 0x5343; // 'S','C'
const CTRL_NEXT: u8 = 1; // advance the playout cursor one frame
const PROTO_VERSION: u8 = 1;

// --- laser_command opcodes (components/laser_command/laser_command.h) ---------
const CMD_GOTO: u8 = 0x01; // x:u16, y:u16
const CMD_LASER: u8 = 0x02; // r:u16, g:u16, b:u16
const CMD_CURVE: u8 = 0x04; // P1,P2,P3:u16 pairs + v_in,v_out:u32

const CENTER: i32 = 0x8000; // galvo mid-scale = zero deflection

#[derive(Copy, Clone, Debug, ValueEnum)]
enum Shape {
    Square,
    Triangle,
    Diamond,
}

#[derive(Parser, Debug)]
#[command(about = "Push one closed-loop frame over TCP and select it over UDP")]
struct Args {
    /// Device IP address (e.g. 172.20.10.2)
    #[arg(long)]
    host: String,

    /// TCP port for the frame data plane
    #[arg(long, default_value_t = 7777)]
    tcp_port: u16,

    /// UDP port for the playout clock
    #[arg(long, default_value_t = 7778)]
    udp_port: u16,

    /// Just advance: send one NEXT tick over UDP and exit, without pushing a
    /// frame. Use this to step through frames already buffered on the device.
    #[arg(long)]
    advance: bool,

    /// Push a scene file (raw .scene wire bytes from `svg2scene --output`) as the
    /// frame instead of a built-in shape. --shape/--size/--intensity are ignored.
    #[arg(long, value_name = "FILE")]
    scene: Option<PathBuf>,

    /// Shape to draw
    #[arg(long, value_enum, default_value_t = Shape::Square)]
    shape: Shape,

    /// Half-extent of the figure in DAC counts (keep well within 0..32767)
    #[arg(long, default_value_t = 18000)]
    size: i32,

    /// Per-channel laser intensity, 0..65535
    #[arg(long, default_value_t = 0x4000)]
    intensity: u16,
}

// --- laser_command encoders (little-endian, matching the firmware codec) ------
fn push_u16(buf: &mut Vec<u8>, v: i32) {
    buf.extend_from_slice(&(v.clamp(0, 0xFFFF) as u16).to_le_bytes());
}
fn push_u32(buf: &mut Vec<u8>, v: u32) {
    buf.extend_from_slice(&v.to_le_bytes());
}

fn goto(buf: &mut Vec<u8>, x: i32, y: i32) {
    buf.push(CMD_GOTO);
    push_u16(buf, x);
    push_u16(buf, y);
}
fn laser(buf: &mut Vec<u8>, r: u16, g: u16, b: u16) {
    buf.push(CMD_LASER);
    push_u16(buf, r as i32);
    push_u16(buf, g as i32);
    push_u16(buf, b as i32);
}
/// Straight edge as a degenerate cubic with evenly spaced control points and
/// v_in = v_out = 0, so the interpolator accelerates off the corner and brakes
/// back to rest into the next one (the same trick as the firmware square demo).
fn straight_curve(buf: &mut Vec<u8>, p0: (i32, i32), p3: (i32, i32)) {
    let c1 = (p0.0 + (p3.0 - p0.0) / 3, p0.1 + (p3.1 - p0.1) / 3);
    let c2 = (p0.0 + 2 * (p3.0 - p0.0) / 3, p0.1 + 2 * (p3.1 - p0.1) / 3);
    buf.push(CMD_CURVE);
    push_u16(buf, c1.0);
    push_u16(buf, c1.1);
    push_u16(buf, c2.0);
    push_u16(buf, c2.1);
    push_u16(buf, p3.0);
    push_u16(buf, p3.1);
    push_u32(buf, 0); // v_in
    push_u32(buf, 0); // v_out
}

fn shape_points(shape: Shape, r: i32) -> Vec<(i32, i32)> {
    let c = CENTER;
    match shape {
        Shape::Square => vec![(c - r, c - r), (c + r, c - r), (c + r, c + r), (c - r, c + r)],
        Shape::Triangle => vec![(c, c + r), (c + r, c - r), (c - r, c - r)],
        Shape::Diamond => vec![(c, c + r), (c + r, c), (c, c - r), (c - r, c)],
    }
}

fn shape_color(shape: Shape, i: u16) -> (u16, u16, u16) {
    match shape {
        Shape::Square => (i, 0, 0),   // red
        Shape::Triangle => (0, i, 0), // green
        Shape::Diamond => (0, 0, i),  // blue
    }
}

/// Build a closed-loop frame: blank → jump to start → beam on → trace the closed
/// polygon → beam off → return to center. It ends in a known rest state so the
/// pump can loop it seamlessly.
fn build_frame(args: &Args) -> Vec<u8> {
    let pts = shape_points(args.shape, args.size);
    let (r, g, b) = shape_color(args.shape, args.intensity);
    let mut f = Vec::new();

    laser(&mut f, 0, 0, 0); // blank travel
    goto(&mut f, pts[0].0, pts[0].1);
    laser(&mut f, r, g, b); // beam on

    for i in 0..pts.len() {
        let p0 = pts[i];
        let p3 = pts[(i + 1) % pts.len()]; // wrap back to the start: closed loop
        straight_curve(&mut f, p0, p3);
    }

    laser(&mut f, 0, 0, 0); // beam off
    goto(&mut f, CENTER, CENTER); // rest at center
    f
}

/// Wrap raw scene bytes (from `svg2scene --output`) into a loopable frame. The
/// scene's first CURVE has an implicit P0 = the field centre, so we prepend a
/// blanked GOTO to centre: this defines that P0 on every loop (not just the first)
/// and blanks the seam between repeats. A trailing blank kills the beam at the end.
fn wrap_scene(scene: &[u8]) -> Vec<u8> {
    let mut f = Vec::with_capacity(scene.len() + 16);
    laser(&mut f, 0, 0, 0); // blank the seam
    goto(&mut f, CENTER, CENTER); // define the scene's implicit P0
    f.extend_from_slice(scene);
    laser(&mut f, 0, 0, 0); // beam off at the end
    f
}

fn build_tcp_packet(payload: &[u8]) -> Vec<u8> {
    let mut pkt = Vec::with_capacity(12 + payload.len());
    pkt.extend_from_slice(&FRAME_MAGIC.to_le_bytes());
    pkt.push(PROTO_VERSION);
    pkt.push(0); // flags (reserved)
    pkt.extend_from_slice(&0u16.to_le_bytes()); // frame id (FIFO playout; tag only)
    pkt.extend_from_slice(&0u16.to_le_bytes()); // reserved
    pkt.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    pkt.extend_from_slice(payload);
    pkt
}

fn build_next_packet() -> Vec<u8> {
    let mut pkt = Vec::with_capacity(4);
    pkt.extend_from_slice(&CTRL_MAGIC.to_le_bytes());
    pkt.push(PROTO_VERSION);
    pkt.push(CTRL_NEXT);
    pkt
}

fn send_next(host: &str, udp_port: u16) -> std::io::Result<()> {
    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.send_to(&build_next_packet(), (host, udp_port))?;
    Ok(())
}

fn main() -> std::io::Result<()> {
    let args = Args::parse();

    // Standalone "just advance": no frame, only a NEXT tick.
    if args.advance {
        send_next(&args.host, args.udp_port)?;
        println!("sent NEXT to {}:{} (advance only)", args.host, args.udp_port);
        return Ok(());
    }

    // Frame payload: a scene file (svg2scene output) or a built-in shape.
    let (payload, what) = match &args.scene {
        Some(path) => {
            let scene = std::fs::read(path)?;
            if scene.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("scene file {} is empty", path.display()),
                ));
            }
            (wrap_scene(&scene), format!("scene {}", path.display()))
        }
        None => (build_frame(&args), format!("shape={:?}", args.shape)),
    };
    let tcp_pkt = build_tcp_packet(&payload);

    // 1. Push the frame over TCP and WAIT for the device's commit ack. The ack
    //    means the frame is fully received and committed — only then is it safe to
    //    advance, so the NEXT tick can't overtake the frame's bytes.
    let mut stream = TcpStream::connect((args.host.as_str(), args.tcp_port))?;
    stream.set_nodelay(true).ok();
    stream.write_all(&tcp_pkt)?;
    stream.flush()?;
    let mut ack = [0u8; 1];
    stream.read_exact(&mut ack)?;
    if ack[0] != FRAME_ACK {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("unexpected commit ack 0x{:02x}", ack[0]),
        ));
    }
    drop(stream);
    println!(
        "sent frame ({}, {} cmd bytes), committed by {}:{}",
        what,
        payload.len(),
        args.host,
        args.tcp_port
    );

    // 2. Now advance the playout cursor to the just-committed frame.
    send_next(&args.host, args.udp_port)?;
    println!("sent NEXT to {}:{} (advance to the frame just pushed)", args.host, args.udp_port);
    Ok(())
}
