//! artnetctl — drive the shadowgraph laser over Art-Net.
//!
//! Sends an Art-Net ArtDmx packet that sets the device's control fixture: the
//! render mode (pattern vs stream) plus the built-in Lissajous settings. The
//! device holds the last values it received, so a single invocation is enough —
//! by default we send a short burst (UDP is lossy) and exit. `--watch` keeps
//! streaming so you can sweep a value live.
//!
//! Channel map (1-based, relative to `--base`), matching components/artnet_control:
//!   1 mode  2 freqX  3 freqY  4 size  5 hue  6 intensity  7 morph  8 phase
//!
//! Examples:
//!   # switch to the streamed scene
//!   artnetctl --host 192.168.1.50 --mode stream
//!   # a red 5:4 Lissajous at half size, slow morph
//!   artnetctl --host 192.168.1.50 --fx 5 --fy 4 --size 0.5 --hue 0 --morph 0.2
//!   # broadcast (default host) and keep sending at 40 Hz
//!   artnetctl --watch --intensity 0.6

use std::net::UdpSocket;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;

use clap::{Parser, ValueEnum};

const ARTNET_PORT: u16 = 6454;
const NUM_CHANNELS: usize = 8;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    /// Trace the built-in Lissajous from the settings below.
    Pattern,
    /// Loop whatever scene the device last received over TCP (port 7777).
    Stream,
}

#[derive(Parser, Debug)]
#[command(name = "artnetctl", version,
          about = "Drive the shadowgraph laser over Art-Net (mode toggle + Lissajous settings).")]
struct Args {
    /// Device address ("ip" or "ip:port"); defaults to the Art-Net broadcast address.
    #[arg(long, default_value = "255.255.255.255")]
    host: String,
    /// Art-Net universe (15-bit Net:SubUni port address) to send on.
    #[arg(long, default_value_t = 1)]
    universe: u16,
    /// 1-based DMX channel of the fixture's first slot.
    #[arg(long, default_value_t = 1)]
    base: u16,

    /// Render mode (channel 1).
    #[arg(long, value_enum, default_value_t = Mode::Pattern)]
    mode: Mode,
    /// Lissajous X frequency ratio, 1..8 (channel 2).
    #[arg(long, default_value_t = 3)]
    fx: u8,
    /// Lissajous Y frequency ratio, 1..8 (channel 3).
    #[arg(long, default_value_t = 2)]
    fy: u8,
    /// Figure size as a fraction of the galvo's linear range, 0..1 (channel 4).
    #[arg(long, default_value_t = 0.8)]
    size: f32,
    /// Color hue in degrees, 0..360 (channel 5).
    #[arg(long, default_value_t = 0.0)]
    hue: f32,
    /// Beam intensity / brightness, 0..1 (channel 6).
    #[arg(long, default_value_t = 0.25)]
    intensity: f32,
    /// Y-phase morph (animation) speed as a fraction of max, 0..1 (channel 7).
    #[arg(long, default_value_t = 0.33)]
    morph: f32,
    /// Static y-phase offset in degrees, 0..360 (channel 8).
    #[arg(long, default_value_t = 0.0)]
    phase: f32,

    /// Number of identical packets to send (UDP is lossy; the device holds state).
    #[arg(long, default_value_t = 3)]
    count: u32,
    /// Keep sending at --hz until interrupted (Ctrl-C), instead of a one-shot burst.
    #[arg(long)]
    watch: bool,
    /// Refresh rate for --watch, in packets per second.
    #[arg(long, default_value_t = 40.0)]
    hz: f64,
    /// Print the DMX channels and the packet bytes without sending.
    #[arg(long)]
    dry_run: bool,
}

/// Quantize a 0..1 fraction to an 8-bit DMX value (rounded, clamped).
fn frac_to_dmx(frac: f32) -> u8 {
    (frac.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// Map a 1..8 frequency ratio to a DMX value the firmware decodes back to it.
/// The firmware decode is `1 + dmx*7/255` (integer), so each ratio owns a band
/// of DMX values; we aim for the band's center so rounding never lands us on a
/// boundary (the center of band d=f-1 is (2d+1)*255/14).
fn freq_to_dmx(f: u8) -> u8 {
    let d = (f.clamp(1, 8) - 1) as u32; // 0..7
    let v = ((2 * d + 1) * 255 + 7) / 14; // round((2d+1)*255/14)
    v.min(255) as u8
}

/// Build the fixture's DMX channels in map order.
fn channels(a: &Args) -> [u8; NUM_CHANNELS] {
    [
        if a.mode == Mode::Stream { 255 } else { 0 },
        freq_to_dmx(a.fx),
        freq_to_dmx(a.fy),
        frac_to_dmx(a.size),
        frac_to_dmx(a.hue / 360.0),
        frac_to_dmx(a.intensity),
        frac_to_dmx(a.morph),
        frac_to_dmx(a.phase / 360.0),
    ]
}

/// Place the fixture channels at `base` within a DMX frame. Art-Net slot counts
/// must be even and at least 2, so the frame is padded up as needed.
fn dmx_frame(base: u16, ch: &[u8; NUM_CHANNELS]) -> Vec<u8> {
    let first = base.max(1) as usize - 1;
    let mut len = first + ch.len();
    if len % 2 == 1 {
        len += 1;
    }
    if len < 2 {
        len = 2;
    }
    let mut dmx = vec![0u8; len];
    dmx[first..first + ch.len()].copy_from_slice(ch);
    dmx
}

/// Encode an Art-Net ArtDmx (OpOutput) packet. OpCode is little-endian; the
/// protocol version and slot length are big-endian, per the Art-Net spec.
fn build_artdmx(seq: u8, universe: u16, dmx: &[u8]) -> Vec<u8> {
    let mut p = Vec::with_capacity(18 + dmx.len());
    p.extend_from_slice(b"Art-Net\0");
    p.extend_from_slice(&[0x00, 0x50]); // OpCode 0x5000 = OpDmx (little-endian)
    p.extend_from_slice(&[0x00, 0x0e]); // ProtVer 14 (big-endian Hi,Lo)
    p.push(seq);
    p.push(0); // physical
    p.push((universe & 0xff) as u8); // SubUni
    p.push(((universe >> 8) & 0x7f) as u8); // Net
    let len = dmx.len() as u16;
    p.push((len >> 8) as u8); // LengthHi (big-endian)
    p.push((len & 0xff) as u8); // LengthLo
    p.extend_from_slice(dmx);
    p
}

/// Normalize a "ip" or "ip:port" host string to a socket address with the
/// default Art-Net port when no port is given.
fn target_addr(host: &str) -> String {
    if host.contains(':') {
        host.to_string()
    } else {
        format!("{host}:{ARTNET_PORT}")
    }
}

fn main() -> ExitCode {
    let args = Args::parse();

    let ch = channels(&args);
    let dmx = dmx_frame(args.base, &ch);
    let target = target_addr(&args.host);

    println!(
        "fixture @ universe {} base ch {}: mode={:?} fx={} fy={} size={:.2} \
         hue={:.0} intensity={:.2} morph={:.2} phase={:.0}",
        args.universe, args.base, args.mode, args.fx, args.fy, args.size, args.hue,
        args.intensity, args.morph, args.phase
    );
    println!(
        "  DMX -> mode={} freqX={} freqY={} size={} hue={} intensity={} morph={} phase={}",
        ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7]
    );

    if args.dry_run {
        let pkt = build_artdmx(1, args.universe, &dmx);
        println!("  packet ({} bytes): {}", pkt.len(),
                 pkt.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
        return ExitCode::SUCCESS;
    }

    let sock = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            eprintln!("bind failed: {e}");
            return ExitCode::FAILURE;
        }
    };
    // Allow broadcast targets (the default host); harmless for unicast.
    let _ = sock.set_broadcast(true);

    let mut seq: u8 = 1;
    let send_one = |seq: u8| -> std::io::Result<()> {
        let pkt = build_artdmx(seq, args.universe, &dmx);
        sock.send_to(&pkt, &target).map(|_| ())
    };

    if args.watch {
        let period = Duration::from_secs_f64(1.0 / args.hz.max(1.0));
        println!("streaming to {target} at {:.0} Hz (Ctrl-C to stop)…", args.hz);
        loop {
            if let Err(e) = send_one(seq) {
                eprintln!("send failed: {e}");
                return ExitCode::FAILURE;
            }
            seq = seq.wrapping_add(1).max(1);
            sleep(period);
        }
    }

    for _ in 0..args.count.max(1) {
        if let Err(e) = send_one(seq) {
            eprintln!("send to {target} failed: {e}");
            return ExitCode::FAILURE;
        }
        seq = seq.wrapping_add(1).max(1);
        sleep(Duration::from_millis(20));
    }
    println!("sent {} packet(s) to {target}", args.count.max(1));
    ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn freq_roundtrips_through_firmware_decode() {
        // Firmware decodes dmx -> 1 + dmx*7/255 (integer). Our encode must land
        // on a value that decodes back to the same ratio.
        for f in 1u8..=8 {
            let dmx = freq_to_dmx(f) as u32;
            let decoded = 1 + dmx * 7 / 255;
            assert_eq!(decoded as u8, f, "ratio {f} did not round-trip (dmx={dmx})");
        }
    }

    #[test]
    fn frac_endpoints() {
        assert_eq!(frac_to_dmx(0.0), 0);
        assert_eq!(frac_to_dmx(1.0), 255);
        assert_eq!(frac_to_dmx(0.5), 128); // 127.5 rounds up
    }

    #[test]
    fn frame_places_channels_at_base_and_is_even() {
        let ch = [10, 20, 30, 40, 50, 60, 70, 80];
        let f = dmx_frame(10, &ch);
        assert_eq!(f.len() % 2, 0);
        assert_eq!(&f[9..17], &ch); // base 10 -> index 9
        assert_eq!(f[0], 0);
    }

    #[test]
    fn artdmx_header_is_wellformed() {
        let pkt = build_artdmx(7, 0x0213, &[1, 2]);
        assert_eq!(&pkt[0..8], b"Art-Net\0");
        assert_eq!(&pkt[8..10], &[0x00, 0x50]); // OpDmx LE
        assert_eq!(&pkt[10..12], &[0x00, 0x0e]); // ProtVer 14 BE
        assert_eq!(pkt[14], 0x13); // SubUni
        assert_eq!(pkt[15], 0x02); // Net
        assert_eq!(&pkt[16..18], &[0x00, 0x02]); // length 2 BE
    }
}
