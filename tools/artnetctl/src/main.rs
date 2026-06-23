//! artnetctl — drive the shadowgraph laser over Art-Net.
//!
//! Sets the device's control fixture: the render mode (pattern vs stream) plus
//! the Lissajous settings and effects. Art-Net/DMX is a *full-frame* protocol —
//! every packet carries the whole universe and the device replaces its state
//! from it — so to behave like "just change what I name", `artnetctl` remembers
//! the last frame it sent (per universe+base, under ~/.cache/artnetctl) and
//! merges only the flags you pass this run on top of it. Unmentioned channels
//! keep their previous values. `--reset` starts from the built-in defaults.
//!
//! The device holds the last frame it received, so one invocation is enough; we
//! send a short burst (UDP is lossy) and exit. `--watch` keeps streaming.
//!
//! Channel map (1-based, relative to `--base`), matching components/artnet_control:
//!   1 mode  2 freqX  3 freqY  4 size  5 hue  6 intensity  7 morph  8 phase
//!   9 blank-width  10 blank-slide  11 color-span  12 color-cycle
//!   13 fx-morph  14 fy-morph  15 freq-depth
//!
//! Examples:
//!   # find the device, switch to the streamed scene
//!   artnetctl --host auto --mode stream
//!   # a red 5:4 Lissajous at half size, slow morph
//!   artnetctl --host shadowgraph.local --fx 5 --fy 4 --size 0.5 --morph 0.2
//!   # now JUST add a sliding blank gap — everything else is left as-is
//!   artnetctl --host shadowgraph.local --blank-width 0.2 --blank-slide 0.3

use std::net::{Ipv4Addr, UdpSocket};
use std::path::PathBuf;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::{Duration, Instant};

use clap::{Parser, ValueEnum};

const ARTNET_PORT: u16 = 6454;
const NUM_CHANNELS: usize = 15;

// Channel indices within the fixture (0-based), matching the firmware decode.
const CH_MODE: usize = 0;
const CH_FX: usize = 1;
const CH_FY: usize = 2;
const CH_SIZE: usize = 3;
const CH_HUE: usize = 4;
const CH_INTENSITY: usize = 5;
const CH_MORPH: usize = 6;
const CH_PHASE: usize = 7;
const CH_BLANK_WIDTH: usize = 8;
const CH_BLANK_SLIDE: usize = 9;
const CH_COLOR_SPAN: usize = 10;
const CH_COLOR_CYCLE: usize = 11;
const CH_FX_MORPH: usize = 12;
const CH_FY_MORPH: usize = 13;
const CH_FREQ_DEPTH: usize = 14;

#[derive(Copy, Clone, Debug, PartialEq, Eq, ValueEnum)]
enum Mode {
    /// Trace the built-in Lissajous from the settings below.
    Pattern,
    /// Loop whatever scene the device last received over TCP (port 7777).
    Stream,
}

#[derive(Parser, Debug)]
#[command(name = "artnetctl", version,
          about = "Drive the shadowgraph laser over Art-Net (mode + Lissajous/effects). \
                   Only the options you pass change; the rest keep their last values.")]
struct Args {
    /// Device address: "ip", "ip:port", "name.local" (mDNS), or "auto" to find it
    /// via ArtPoll discovery. Defaults to the Art-Net broadcast address.
    #[arg(long, default_value = "255.255.255.255")]
    host: String,
    /// Art-Net universe (15-bit Net:SubUni port address) to send on.
    #[arg(long, default_value_t = 1)]
    universe: u16,
    /// 1-based DMX channel of the fixture's first slot.
    #[arg(long, default_value_t = 1)]
    base: u16,

    // --- Fixture settings. Each is optional: unset = leave that channel at its
    //     last-sent value (merged from the saved frame). ---
    /// Render mode: pattern | stream (channel 1).
    #[arg(long, value_enum)]
    mode: Option<Mode>,
    /// Lissajous X base frequency ratio, 1..8 (channel 2).
    #[arg(long)]
    fx: Option<u8>,
    /// Lissajous Y base frequency ratio, 1..8 (channel 3).
    #[arg(long)]
    fy: Option<u8>,
    /// Figure size as a fraction of the galvo's linear range, 0..1 (channel 4).
    #[arg(long)]
    size: Option<f32>,
    /// Base color hue in degrees, 0..360 (channel 5).
    #[arg(long)]
    hue: Option<f32>,
    /// Beam intensity / brightness, 0..1 (channel 6).
    #[arg(long)]
    intensity: Option<f32>,
    /// Y-phase morph (animation) speed as a fraction of max, 0..1 (channel 7).
    #[arg(long)]
    morph: Option<f32>,
    /// Static y-phase offset in degrees, 0..360 (channel 8).
    #[arg(long)]
    phase: Option<f32>,
    /// Sliding blank-gap width as a fraction of max, 0..1 (channel 9; 0 = off).
    #[arg(long)]
    blank_width: Option<f32>,
    /// Blank-gap slide speed as a fraction of max, 0..1 (channel 10).
    #[arg(long)]
    blank_slide: Option<f32>,
    /// Hue spread along the curve as a fraction of max, 0..1 (channel 11; 0 = constant).
    #[arg(long)]
    color_span: Option<f32>,
    /// Color rotation-over-time speed as a fraction of max, 0..1 (channel 12).
    #[arg(long)]
    color_cycle: Option<f32>,
    /// fx morph (oscillation) rate as a fraction of max, 0..1 (channel 13).
    #[arg(long)]
    fx_morph: Option<f32>,
    /// fy morph (oscillation) rate as a fraction of max, 0..1 (channel 14).
    #[arg(long)]
    fy_morph: Option<f32>,
    /// fx/fy morph depth as a fraction of max, 0..1 (channel 15).
    #[arg(long)]
    freq_depth: Option<f32>,

    /// Ignore the saved frame and start from the built-in defaults before applying
    /// any options above.
    #[arg(long)]
    reset: bool,

    /// Number of identical packets to send (UDP is lossy; the device holds state).
    #[arg(long, default_value_t = 3)]
    count: u32,
    /// Keep sending at --hz until interrupted (Ctrl-C), instead of a one-shot burst.
    #[arg(long)]
    watch: bool,
    /// Refresh rate for --watch, in packets per second.
    #[arg(long, default_value_t = 40.0)]
    hz: f64,
    /// Print the resulting frame without sending (does not update the saved state).
    #[arg(long)]
    dry_run: bool,

    /// Discover Art-Net nodes via ArtPoll, print them, and exit (sends no DMX).
    #[arg(long)]
    discover: bool,
    /// Substring a node's name must contain to be picked by --host auto /
    /// --discover (case-insensitive).
    #[arg(long, default_value = "shadowgraph")]
    match_name: String,
    /// How long to listen for ArtPollReply during discovery, in milliseconds.
    #[arg(long, default_value_t = 1500)]
    discover_ms: u64,
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

/// The baseline frame used on first run / `--reset`: the plain morphing Lissajous
/// (pattern, 3:2, ~full size, 25% intensity, gentle morph) with effects off.
fn default_channels() -> [u8; NUM_CHANNELS] {
    let mut c = [0u8; NUM_CHANNELS];
    c[CH_FX] = freq_to_dmx(3);
    c[CH_FY] = freq_to_dmx(2);
    c[CH_SIZE] = frac_to_dmx(0.8);
    c[CH_INTENSITY] = frac_to_dmx(0.25);
    c[CH_MORPH] = frac_to_dmx(0.33);
    c
}

/// Apply the options the user actually passed onto `ch`, returning a human list
/// of what changed (for the status line).
fn apply_overrides(ch: &mut [u8; NUM_CHANNELS], a: &Args) -> Vec<String> {
    let mut changed: Vec<String> = Vec::new();
    if let Some(m) = a.mode {
        ch[CH_MODE] = if m == Mode::Stream { 255 } else { 0 };
        changed.push(format!("mode={}", if m == Mode::Stream { "stream" } else { "pattern" }));
    }
    if let Some(v) = a.fx { ch[CH_FX] = freq_to_dmx(v); changed.push(format!("fx={v}")); }
    if let Some(v) = a.fy { ch[CH_FY] = freq_to_dmx(v); changed.push(format!("fy={v}")); }
    if let Some(v) = a.size { ch[CH_SIZE] = frac_to_dmx(v); changed.push(format!("size={v}")); }
    if let Some(v) = a.hue { ch[CH_HUE] = frac_to_dmx(v / 360.0); changed.push(format!("hue={v}")); }
    if let Some(v) = a.intensity { ch[CH_INTENSITY] = frac_to_dmx(v); changed.push(format!("intensity={v}")); }
    if let Some(v) = a.morph { ch[CH_MORPH] = frac_to_dmx(v); changed.push(format!("morph={v}")); }
    if let Some(v) = a.phase { ch[CH_PHASE] = frac_to_dmx(v / 360.0); changed.push(format!("phase={v}")); }
    if let Some(v) = a.blank_width { ch[CH_BLANK_WIDTH] = frac_to_dmx(v); changed.push(format!("blank-width={v}")); }
    if let Some(v) = a.blank_slide { ch[CH_BLANK_SLIDE] = frac_to_dmx(v); changed.push(format!("blank-slide={v}")); }
    if let Some(v) = a.color_span { ch[CH_COLOR_SPAN] = frac_to_dmx(v); changed.push(format!("color-span={v}")); }
    if let Some(v) = a.color_cycle { ch[CH_COLOR_CYCLE] = frac_to_dmx(v); changed.push(format!("color-cycle={v}")); }
    if let Some(v) = a.fx_morph { ch[CH_FX_MORPH] = frac_to_dmx(v); changed.push(format!("fx-morph={v}")); }
    if let Some(v) = a.fy_morph { ch[CH_FY_MORPH] = frac_to_dmx(v); changed.push(format!("fy-morph={v}")); }
    if let Some(v) = a.freq_depth { ch[CH_FREQ_DEPTH] = frac_to_dmx(v); changed.push(format!("freq-depth={v}")); }
    changed
}

/// Cache file holding the last frame sent for a given universe+base.
fn state_path(universe: u16, base: u16) -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    let mut p = PathBuf::from(home);
    p.push(".cache");
    p.push("artnetctl");
    p.push(format!("u{universe}_b{base}.dmx"));
    Some(p)
}

/// Load the last frame sent (per universe+base), or the defaults if there's none.
fn load_channels(universe: u16, base: u16) -> [u8; NUM_CHANNELS] {
    if let Some(p) = state_path(universe, base) {
        if let Ok(bytes) = std::fs::read(&p) {
            if bytes.len() == NUM_CHANNELS {
                let mut c = [0u8; NUM_CHANNELS];
                c.copy_from_slice(&bytes);
                return c;
            }
        }
    }
    default_channels()
}

/// Persist the frame just sent so the next run can merge onto it.
fn save_channels(universe: u16, base: u16, ch: &[u8; NUM_CHANNELS]) {
    if let Some(p) = state_path(universe, base) {
        if let Some(dir) = p.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let _ = std::fs::write(&p, ch);
    }
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

/// A node learned from an ArtPollReply.
struct Node {
    ip: Ipv4Addr,
    short: String,
    long: String,
}

/// Build an ArtPoll discovery query: header + OpPoll + protocol version.
fn build_artpoll() -> Vec<u8> {
    let mut p = b"Art-Net\0".to_vec();
    p.extend_from_slice(&[0x00, 0x20]); // OpCode 0x2000 = OpPoll (little-endian)
    p.extend_from_slice(&[0x00, 0x0e]); // ProtVer 14 (big-endian)
    p.push(0x00); // TalkToMe
    p.push(0x00); // Priority
    p
}

/// Read a null-terminated name field out of an ArtPollReply.
fn read_cstr(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

/// Parse an ArtPollReply into the node it describes (IP at offset 10, ShortName
/// at 26, LongName at 44 — see components/artnet_control).
fn parse_pollreply(pkt: &[u8]) -> Option<Node> {
    if pkt.len() < 108 || &pkt[0..8] != b"Art-Net\0" {
        return None;
    }
    if u16::from_le_bytes([pkt[8], pkt[9]]) != 0x2100 {
        return None;
    }
    Some(Node {
        ip: Ipv4Addr::new(pkt[10], pkt[11], pkt[12], pkt[13]),
        short: read_cstr(&pkt[26..44]),
        long: read_cstr(&pkt[44..108]),
    })
}

/// Broadcast an ArtPoll and collect the unique nodes that reply within `timeout`.
fn discover(timeout: Duration) -> std::io::Result<Vec<Node>> {
    let sock = UdpSocket::bind("0.0.0.0:0")?;
    sock.set_broadcast(true)?;
    sock.set_read_timeout(Some(Duration::from_millis(250)))?;
    sock.send_to(&build_artpoll(), ("255.255.255.255", ARTNET_PORT))?;

    let mut nodes: Vec<Node> = Vec::new();
    let mut buf = [0u8; 1024];
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match sock.recv_from(&mut buf) {
            Ok((n, _src)) => {
                if let Some(node) = parse_pollreply(&buf[..n]) {
                    if !nodes.iter().any(|x| x.ip == node.ip) {
                        nodes.push(node);
                    }
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e),
        }
    }
    Ok(nodes)
}

/// Discover and pick the node whose name contains `want` (case-insensitive), or
/// the sole node if there's exactly one. Returns its "ip:port".
fn resolve_auto(want: &str, timeout: Duration) -> Result<String, String> {
    let nodes = discover(timeout).map_err(|e| format!("discovery failed: {e}"))?;
    if nodes.is_empty() {
        return Err("no Art-Net nodes replied to ArtPoll".into());
    }
    let want_lc = want.to_lowercase();
    let chosen = nodes
        .iter()
        .find(|n| n.short.to_lowercase().contains(&want_lc) || n.long.to_lowercase().contains(&want_lc))
        .or(if nodes.len() == 1 { nodes.first() } else { None });
    match chosen {
        Some(n) => {
            println!("discovered \"{}\" at {}", n.short, n.ip);
            Ok(format!("{}:{}", n.ip, ARTNET_PORT))
        }
        None => Err(format!(
            "{} nodes replied but none match \"{want}\"; pass --host <ip>",
            nodes.len()
        )),
    }
}

fn main() -> ExitCode {
    let args = Args::parse();

    // Discovery-only mode: broadcast an ArtPoll, list who answers, and exit.
    if args.discover {
        match discover(Duration::from_millis(args.discover_ms)) {
            Ok(nodes) if nodes.is_empty() => println!("no Art-Net nodes replied to ArtPoll"),
            Ok(nodes) => {
                println!("discovered {} node(s):", nodes.len());
                for n in &nodes {
                    println!("  {:<15} {}  ({})", n.ip.to_string(), n.short, n.long);
                }
            }
            Err(e) => {
                eprintln!("discovery failed: {e}");
                return ExitCode::FAILURE;
            }
        }
        return ExitCode::SUCCESS;
    }

    // Merge this run's options onto the last frame sent (or the defaults).
    let mut ch = if args.reset {
        default_channels()
    } else {
        load_channels(args.universe, args.base)
    };
    let changed = apply_overrides(&mut ch, &args);

    if args.reset {
        print!("reset to defaults");
        if !changed.is_empty() {
            print!(", then {}", changed.join(" "));
        }
        println!();
    } else if changed.is_empty() {
        println!(
            "no settings given — resending the last frame for universe {} base {}",
            args.universe, args.base
        );
    } else {
        println!("updating: {}", changed.join(" "));
    }
    let dmx_dump = ch.iter().map(|b| b.to_string()).collect::<Vec<_>>().join(" ");
    println!("frame (universe {} base {}): {dmx_dump}", args.universe, args.base);

    let dmx = dmx_frame(args.base, &ch);

    if args.dry_run {
        let pkt = build_artdmx(1, args.universe, &dmx);
        println!("packet ({} bytes): {}", pkt.len(),
                 pkt.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" "));
        return ExitCode::SUCCESS; // preview only — don't persist
    }

    // Resolve the target: discover it via ArtPoll when --host auto.
    let target = if args.host == "auto" {
        match resolve_auto(&args.match_name, Duration::from_millis(args.discover_ms)) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("{e}");
                return ExitCode::FAILURE;
            }
        }
    } else {
        target_addr(&args.host)
    };

    let sock = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            eprintln!("bind failed: {e}");
            return ExitCode::FAILURE;
        }
    };
    // Allow broadcast targets (the default host); harmless for unicast.
    let _ = sock.set_broadcast(true);

    // Persist the frame we're about to send so the next run merges onto it.
    save_channels(args.universe, args.base, &ch);

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
        let ch = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let f = dmx_frame(10, &ch);
        assert_eq!(f.len() % 2, 0);
        assert_eq!(&f[9..9 + NUM_CHANNELS], &ch); // base 10 -> index 9
        assert_eq!(f[0], 0);
    }

    #[test]
    fn overrides_only_touch_named_channels() {
        // The core "just write updates" property: passing one flag must change
        // exactly its channel and leave the rest of the frame untouched.
        let mut ch = default_channels();
        let before = ch;
        let args = Args::parse_from(["artnetctl", "--size", "0.5"]);
        let changed = apply_overrides(&mut ch, &args);

        assert_eq!(changed.len(), 1);
        for i in 0..NUM_CHANNELS {
            if i == CH_SIZE {
                assert_ne!(ch[i], before[i]);
            } else {
                assert_eq!(ch[i], before[i], "channel {i} should be untouched");
            }
        }
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

    #[test]
    fn artpoll_is_wellformed() {
        let p = build_artpoll();
        assert_eq!(&p[0..8], b"Art-Net\0");
        assert_eq!(&p[8..10], &[0x00, 0x20]); // OpPoll LE
        assert_eq!(&p[10..12], &[0x00, 0x0e]); // ProtVer 14 BE
    }

    #[test]
    fn pollreply_parses_ip_and_names() {
        // Hand-build the fields the firmware fills (see artnet_pollreply_build).
        let mut pkt = vec![0u8; 239];
        pkt[0..8].copy_from_slice(b"Art-Net\0");
        pkt[8] = 0x00;
        pkt[9] = 0x21; // OpPollReply, little-endian
        pkt[10..14].copy_from_slice(&[192, 168, 1, 50]);
        pkt[26..26 + 11].copy_from_slice(b"shadowgraph");
        pkt[44..44 + 27].copy_from_slice(b"shadowgraph laser projector");

        let node = parse_pollreply(&pkt).expect("valid reply should parse");
        assert_eq!(node.ip, Ipv4Addr::new(192, 168, 1, 50));
        assert_eq!(node.short, "shadowgraph");
        assert_eq!(node.long, "shadowgraph laser projector");

        // Wrong opcode (an ArtPoll, say) is not a reply.
        pkt[9] = 0x20;
        assert!(parse_pollreply(&pkt).is_none());
    }
}
