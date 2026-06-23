# ildaplay

Stream **ILDA frames** to the shadowgraph projector at a frame rate.

`ildaplay` reads one or more ILDA files, concatenates their frames into a
playlist, and sends them to the device (TCP, port 7777) in order at a fixed FPS.
The projector loops whichever frame it last received — its scene buffer persists
— so between sends it keeps drawing the active frame, and each send swaps to the
next. This is the playout half of the pipeline; [`svg2scene`](../svg2scene) makes
the frames.

## Usage

```sh
# single frame: sent once and held (projector draws it until replaced)
cargo run --manifest-path tools/ildaplay/Cargo.toml -- --host <device-ip> logo.ild

# animation: many frames at 12 fps, looping (Ctrl-C to stop)
cargo run --manifest-path tools/ildaplay/Cargo.toml -- \
    --host <device-ip> --fps 12 frame00.ild frame01.ild frame02.ild …

# a single multi-frame .ild also works; --once plays through and stops
cargo run --manifest-path tools/ildaplay/Cargo.toml -- --host <device-ip> --once anim.ild
```

| flag | meaning |
|------|---------|
| `<FILES>...` | ILDA files; all frames are concatenated into the playlist in order |
| `--host` | device address (`ip` or `ip:port`; default port 7777) |
| `--fps` | animation frame rate (default 12) |
| `--once` | play the playlist once instead of looping |

## How it works

Per animation frame, `ildaplay` sends that frame's ILDA section plus a 0-record
terminating header and waits for the device's 1-byte ACK, then sleeps to hold the
frame period. The frame rate is the *animation* rate (how fast frames swap); the
device redraws each held frame far faster (point-rate ÷ points-per-frame), which
is what keeps it flicker-free between swaps.

Only true-colour ILDA (format 5 = 2D, format 4 = 3D) is supported — the formats
the projector draws. Indexed/palette files (formats 0/1/2) are rejected; convert
them to true colour first.
