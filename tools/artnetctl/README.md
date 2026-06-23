# artnetctl

Drive the shadowgraph laser over **Art-Net** (DMX-over-UDP, port 6454): toggle
between the built-in **Lissajous pattern** and the **streamed scene**, and tweak
the pattern's shape, size, color, and motion.

The device listens for Art-Net on one DMX universe and maps a small block of
channels to its control state (see [`components/artnet_control`](../../components/artnet_control)).
It **holds** the last values it received, so a single invocation is enough — by
default `artnetctl` sends a short burst (UDP is lossy) and exits.

## Usage

```sh
# switch to the streamed scene (then stream frames with ildaplay)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host <device-ip> --mode stream

# back to a pattern: a red 5:4 Lissajous at half size with a slow morph
cargo run --manifest-path tools/artnetctl/Cargo.toml -- \
    --host <device-ip> --mode pattern --fx 5 --fy 4 --size 0.5 --hue 0 --morph 0.2

# no --host => broadcast to the whole subnet; --watch keeps sending so you can
# sweep a value live (Ctrl-C to stop)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --watch --intensity 0.6

# see exactly what would go on the wire without sending
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --mode stream --dry-run
```

## Options

| flag | meaning | default |
|------|---------|---------|
| `--host` | device address (`ip` or `ip:port`); the default is the Art-Net broadcast address | `255.255.255.255` |
| `--universe` | Art-Net universe (15-bit Net:SubUni port address) | `1` |
| `--base` | 1-based DMX channel of the fixture's first slot | `1` |
| `--mode` | `pattern` (Lissajous) or `stream` (looped scene) | `pattern` |
| `--fx` / `--fy` | Lissajous X/Y frequency ratios, 1–8 | `3` / `2` |
| `--size` | figure size as a fraction of the galvo's linear range, 0–1 | `0.8` |
| `--hue` | color hue in degrees, 0–360 | `0` |
| `--intensity` | beam brightness, 0–1 | `0.25` |
| `--morph` | y-phase morph (animation) speed as a fraction of max, 0–1 | `0.33` |
| `--phase` | static y-phase offset in degrees, 0–360 | `0` |
| `--count` | identical packets per invocation (resilience to UDP loss) | `3` |
| `--watch` | keep sending at `--hz` until Ctrl-C, instead of a one-shot burst | off |
| `--hz` | refresh rate for `--watch` | `40` |
| `--dry-run` | print the DMX channels + packet bytes without sending | off |

## DMX channel map

The fixture occupies 8 channels, starting at `--base` (1-based) within the
universe. This matches the firmware decode in `components/artnet_control`:

| channel | parameter | mapping |
|---------|-----------|---------|
| 1 | mode | `< 128` = pattern, `>= 128` = stream |
| 2 | freq X | `1 + value*7/255` → ratio 1–8 |
| 3 | freq Y | ratio 1–8 |
| 4 | size | `0 .. ARTNET_AMP_MAX` (galvo linear range) |
| 5 | hue | `0 .. 360°` |
| 6 | intensity | `0 .. 1` |
| 7 | morph rate | `0 .. ARTNET_MORPH_MAX_RAD_S` (y-phase precession) |
| 8 | phase offset | `0 .. 2π` |

Because these are plain DMX channels on a standard universe, any Art-Net source
(QLC+, a lighting console, Resolume) can drive the projector too — patch an
8-channel generic fixture at the base channel and the faders line up with the
table above. Sending Art-Net never requires a multicast join; `artnetctl`
broadcasts or unicasts straight to the device.
