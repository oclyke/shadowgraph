# artnetctl

Drive the shadowgraph laser over **Art-Net** (DMX-over-UDP, port 6454): toggle
between the built-in **Lissajous pattern** and the **streamed scene**, and tweak
the pattern's shape, size, color, motion, and "aliasing" effects.

The device listens for Art-Net on one DMX universe and maps a block of channels
to its control state (see [`components/artnet_control`](../../components/artnet_control)).
It **holds** the last frame it received, so a single invocation is enough — by
default `artnetctl` sends a short burst (UDP is lossy) and exits.

## How settings merge (you only change what you name)

Art-Net/DMX is a **full-frame** protocol: every packet carries the whole universe
and the device replaces its state from it. So that you don't have to respecify
everything each time, `artnetctl` **remembers the last frame it sent** (per
universe + base, under `~/.cache/artnetctl/`) and merges only the options you pass
this run on top of it — unmentioned channels keep their previous values, exactly
like nudging one fader on a desk. `--reset` starts from the built-in defaults
before applying your options; `--dry-run` previews without sending or saving.

## Finding the device

You don't have to know the IP. Three options:

```sh
# 1. list everything that answers an Art-Net ArtPoll
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --discover

# 2. let the tool discover + target the shadowgraph node automatically
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host auto --mode stream

# 3. mDNS: the device advertises shadowgraph.local (works as --host for ildaplay too)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host shadowgraph.local --mode stream
```

`--discover` broadcasts an ArtPoll and prints each node's IP + name; `--host auto`
does the same and picks the node whose name matches `--match-name` (default
`shadowgraph`), or the only node if there's just one.

## Usage

```sh
# switch to the streamed scene (then stream frames with ildaplay)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host auto --mode stream

# a red 5:4 Lissajous at half size with a slow morph
cargo run --manifest-path tools/artnetctl/Cargo.toml -- \
    --host shadowgraph.local --mode pattern --fx 5 --fy 4 --size 0.5 --hue 0 --morph 0.2

# now add ONLY a sliding blank gap (the "aliasing" artifact) — fx/fy/size/hue
# from the line above are kept; we just set channels 9 and 10
cargo run --manifest-path tools/artnetctl/Cargo.toml -- \
    --host shadowgraph.local --blank-width 0.2 --blank-slide 0.3

# go wild: color sweeping along the curve + both frequencies morphing
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host shadowgraph.local \
    --color-span 0.5 --color-cycle 0.3 --fx-morph 0.4 --fy-morph 0.25 --freq-depth 0.6

# --watch keeps sending so you can sweep a value live (Ctrl-C to stop)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host shadowgraph.local --watch --intensity 0.6

# preview the merged frame without sending or saving
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --blank-width 0.3 --dry-run
```

## Options

| flag | meaning | default |
|------|---------|---------|
| `--host` | device address: `ip`, `ip:port`, `name.local` (mDNS), or `auto` (ArtPoll discovery); default is the Art-Net broadcast address | `255.255.255.255` |
| `--discover` | broadcast an ArtPoll, print the nodes that answer, and exit | off |
| `--match-name` | substring a node's name must contain for `--host auto` / `--discover` | `shadowgraph` |
| `--discover-ms` | how long to listen for ArtPollReply | `1500` |
| `--universe` | Art-Net universe (15-bit Net:SubUni port address) | `1` |
| `--base` | 1-based DMX channel of the fixture's first slot | `1` |
| `--reset` | start from the built-in defaults before applying options | off |
| `--count` | identical packets per invocation (resilience to UDP loss) | `3` |
| `--watch` | keep sending at `--hz` until Ctrl-C, instead of a one-shot burst | off |
| `--hz` | refresh rate for `--watch` | `40` |
| `--dry-run` | print the merged frame without sending or saving | off |

Fixture settings — each is **optional**; omit it to keep that channel's last-sent
value (see "How settings merge" above):

| flag | meaning | range |
|------|---------|-------|
| `--mode` | `pattern` (Lissajous) or `stream` (looped scene) | enum |
| `--fx` / `--fy` | Lissajous X/Y base frequency ratios | 1–8 |
| `--size` | figure size, fraction of the galvo's linear range | 0–1 |
| `--hue` | base color hue, degrees | 0–360 |
| `--intensity` | beam brightness | 0–1 |
| `--morph` | y-phase morph (animation) speed, fraction of max | 0–1 |
| `--phase` | static y-phase offset, degrees | 0–360 |
| `--blank-width` | sliding blank-gap size, fraction of max (0 = off) | 0–1 |
| `--blank-slide` | blank-gap slide speed, fraction of max | 0–1 |
| `--color-span` | hue spread **along the curve**, fraction of max (0 = constant) | 0–1 |
| `--color-cycle` | color rotation-over-time speed, fraction of max | 0–1 |
| `--fx-morph` / `--fy-morph` | per-axis frequency morph (oscillation) rate, fraction of max | 0–1 |
| `--freq-depth` | how far fx/fy swing while morphing, fraction of max | 0–1 |

## DMX channel map

The fixture occupies 15 channels, starting at `--base` (1-based) within the
universe. This matches the firmware decode in `components/artnet_control`:

| channel | parameter | mapping |
|---------|-----------|---------|
| 1 | mode | `< 128` = pattern, `>= 128` = stream |
| 2 | freq X | `1 + value*7/255` → base ratio 1–8 |
| 3 | freq Y | base ratio 1–8 |
| 4 | size | `0 .. ARTNET_AMP_MAX` (galvo linear range) |
| 5 | hue | `0 .. 360°` (base color) |
| 6 | intensity | `0 .. 1` |
| 7 | morph rate | `0 .. ARTNET_MORPH_MAX_RAD_S` (y-phase precession) |
| 8 | phase offset | `0 .. 2π` |
| 9 | blank width | `0 .. ARTNET_BLANK_WIDTH_MAX` — sliding blank gap size (0 = off) |
| 10 | blank slide rate | `0 .. ARTNET_BLANK_SLIDE_MAX` loop-fractions/sec the gap slides |
| 11 | color-vs-t span | `0 .. ARTNET_COLOR_SPAN_MAX°` of hue spread along the curve |
| 12 | color cycle rate | `0 .. ARTNET_COLOR_CYCLE_MAX°`/sec time rotation |
| 13 | freq X morph rate | `0 .. ARTNET_FREQ_MORPH_RATE_MAX` Hz (fx oscillates) |
| 14 | freq Y morph rate | `0 .. ARTNET_FREQ_MORPH_RATE_MAX` Hz (fy oscillates) |
| 15 | freq morph depth | `0 .. ARTNET_FREQ_MORPH_DEPTH_MAX` (± added to fx/fy) |

The "aliasing" artifact is channels 9–10: a gap in the parametric domain `t`
where the beam blanks, sliding around the figure. Channels 11–12 make the color a
function of `t` that also rotates over time; 13–15 let both frequencies morph for
evolving figures (a morphed figure no longer closes, so the device blanks the
loop seam to hide the flyback).

Because these are plain DMX channels on a standard universe, any Art-Net source
(QLC+, a lighting console, Resolume) can drive the projector too — patch a
15-channel generic fixture at the base channel and the faders line up with the
table above. Sending Art-Net never requires a multicast join; `artnetctl`
broadcasts or unicasts straight to the device.
