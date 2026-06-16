# frame_send

Host side of the shadowgraph frame-streaming demo. Builds **one closed-loop
frame**, pushes it to the device over TCP (the frame data plane), **waits for the
device's commit ack**, then advances the device to it with a `NEXT` tick over UDP
(the playout clock). The ack matters: a local `write()` finishing doesn't mean the
device has the frame, so without it the UDP tick can overtake the TCP bytes and
advance into an empty queue. Playout is **relative and strictly FIFO**: `NEXT`
steps the device's cursor one frame in commit order, so the host never tracks
which frame is showing, and frame ids are not used to address frames. Run the tool
repeatedly to step an animation — each push becomes the displayed frame. The wire
formats match `components/frame_stream/frame_stream.c`; the design is in
`docs/FRAME_STREAMING.md`.

## Build & run

```sh
# from the repo root
cargo run --manifest-path tools/frame_send/Cargo.toml -- --host 172.20.10.2
```

Or build once and call the binary directly:

```sh
cargo build --manifest-path tools/frame_send/Cargo.toml --release
./tools/frame_send/target/release/frame_send --host 172.20.10.2 --shape triangle
```

Push a few frames, then step through what's buffered without sending more:

```sh
frame_send --host 172.20.10.2 --shape square      # push + advance to it
frame_send --host 172.20.10.2 --shape triangle    # push + advance to it
frame_send --host 172.20.10.2 --advance           # just advance (past the end → blank)
```

The device must be reachable on the network it joined (see the `got ip:` line in
its serial log) and `ENABLE_FRAME_STREAM` must be on in the firmware (it is by
default).

## Options

| Flag | Default | Meaning |
|------|---------|---------|
| `--host` | *(required)* | Device IP address (e.g. `172.20.10.2`) |
| `--tcp-port` | `7777` | TCP port for the frame data plane |
| `--udp-port` | `7778` | UDP port for the playout clock |
| `--advance` | off | Just advance: send one `NEXT` tick and exit, **without** pushing a frame. Steps through frames already buffered on the device. |
| `--shape` | `square` | `square` \| `triangle` \| `diamond` |
| `--size` | `18000` | Half-extent of the figure in DAC counts (keep well within `0..32767`) |
| `--intensity` | `0x4000` | Per-channel laser intensity, `0..65535` |

## What it sends

A frame is a blob of `laser_command` records:

1. `LASER 0,0,0` — blank for travel
2. `GOTO` to the first vertex
3. `LASER r,g,b` — beam on (colour per shape: square=red, triangle=green, diamond=blue)
4. one straight `CURVE` per polygon edge, closing back to the first vertex
   (`v_in = v_out = 0`, so the interpolator accelerates off each corner and
   brakes into the next — the same trick as the firmware square demo)
5. `LASER 0,0,0` then `GOTO` center — end in a known rest state so the loop is seamless

The device replies with a 1-byte commit ack (`0x06`); the tool waits for it, then
a 4-byte UDP `NEXT` packet advances the device's playout cursor to the frame.

## Wire formats (kept in sync with the firmware)

```
TCP frame:  [u16 magic 0x4652][u8 ver=1][u8 flags][u16 id][u16 rsv][u32 len][payload]
TCP ack:    [u8 0x06]   (device -> host, after each frame commits)
UDP NEXT:   [u16 magic 0x5343][u8 ver=1][u8 type=1]
```

All fields little-endian. `payload` is `len` bytes of back-to-back
`laser_command` records (opcodes in `components/laser_command/laser_command.h`).
