# shadowgraph

Firmware for a laser projector built on the [LaserScanner](https://github.com/oclyke/LaserScanner)
hardware: an ESP32 drives two galvanometer axes and an RGB laser from a
time-ordered command stream. Renderers push `goto` / `laser` / `dwell` commands
into a lock-free queue; a hardware-timer-paced consumer task clocks them out to
the DACs.

## Getting Started

```sh
export SHADOWGRAPH_HOST="172.20.10.2"
./play.sh --once tools/svg2scene/examples/chicken.svg

# discover the laser on the network
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --discover --discover-ms 3000 

# send some commands
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host 172.20.10.2 --mode pattern --intensity 0.25 --fx-morph 0.3 --freq-depth 0.1 --blank-width 0.25 --blank-slide 0.2 --color-cycle 0.1 --color-span 0.2

# go to stream mode for ILDA format control
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host 172.20.10.2 --mode stream
```

## Hardware

- **MCU:** original ESP32 (Xtensa LX6, dual-core)
- **Galvos:** 2× DAC8871 (16-bit) on SPI2 / HSPI
- **Laser:** 1× DAC80004 (quad 16-bit) on SPI3 / VSPI, channels A/B/C = R/G/B

| Signal | GPIO | | Signal | GPIO |
|--------|------|--|--------|------|
| Galvo MOSI (shared) | IO13 | | Laser MOSI | IO23 |
| Galvo SCLK (shared) | IO14 | | Laser SCLK | IO18 |
| Galvo X — CS / LDAC / RST | IO21 / IO19 / IO22 | | Laser SYNC (CS) | IO26 |
| Galvo Y — CS / LDAC / RST | IO16 / IO15 / IO17 | | Laser LDAC / CLR | IO27 / IO25 |

Pin assignments were traced from the LaserScanner Eagle schematic. The galvo
DACs share one SPI bus with per-device chip selects; the laser DAC is on its own
bus.

## Repository layout

```
shadowgraph/
├── CMakeLists.txt          # top-level ESP-IDF project
├── sdkconfig.defaults      # pins the target to esp32
├── env.sh                  # sets IDF_PATH and sources the ESP-IDF environment
├── main/                   # application entry point (app_main)
├── components/
│   ├── driver_dac8871/     # DAC8871 galvo driver (ESP-IDF v6 SPI/GPIO wrapper)
│   ├── driver_dacx0004/    # DAC80004 laser driver (ESP-IDF v6 SPI/GPIO wrapper)
│   ├── point_ring/         # SPSC lock-free ring of ILDA points
│   ├── laser_engine/       # point ring + fixed-rate timer-ISR DAC consumer
│   ├── point_stream/       # triple-buffered scene store + TCP ILDA receiver
│   ├── artnet_control/     # Art-Net receiver -> render mode + Lissajous settings
│   └── wifi_ap/            # WiFi bring-up (SoftAP or station)
├── tools/
│   ├── svg2scene/          # SVG -> ILDA scene
│   ├── ildaplay/           # stream ILDA frames to the device
│   └── artnetctl/          # Art-Net control: mode toggle + Lissajous settings
└── third-party/github.com/
    ├── espressif/esp-idf           # pinned ESP-IDF v6.0.1
    ├── oclyke/driver-DAC8871       # vendored galvo driver source
    ├── oclyke/driver-DACx0004      # vendored laser driver source
    └── google/googletest           # host-test framework (v1.15.2)
```

Third-party dependencies are vendored as git submodules under
`third-party/github.com/<org>/<repo>`.

## Setup

Clone and fetch the pinned dependencies, then install the toolchain once:

```sh
git clone https://github.com/oclyke/shadowgraph.git
cd shadowgraph

# Fetch all submodules (ESP-IDF and its nested submodules, the DAC drivers,
# and googletest). The ESP-IDF tree is large, so this can take a while.
git submodule update --init --recursive

# Set IDF_PATH and put idf.py on PATH (sources ESP-IDF's export.sh)
source env.sh

# Install the esp32 toolchain
$IDF_PATH/install.sh esp32
```

## Every new shell session

```sh
. env.sh
```

This sets `IDF_PATH` to the pinned ESP-IDF submodule and puts `idf.py` on your
`PATH`.

## Build

```sh
idf.py build
```

## Flash and monitor

```sh
idf.py -p /dev/tty.usbserial-XXXX flash monitor
```

## Configure

```sh
idf.py menuconfig
```

## Clean

```sh
idf.py fullclean
```

## Components

- **driver_dac8871 / driver_dacx0004** — thin ESP-IDF v6 wrappers around the
  vendored DAC driver sources. They supply the SPI/GPIO platform implementation
  (the in-tree `if_imp/esp32` code targets older IDF and is not used).
- **point_ring** — single-producer/single-consumer lock-free ring of
  `laser_point_t`, an ILDA-format point (signed-16-bit X/Y, 8-bit RGB, status).
  Each point is published atomically via a release/acquire handoff.
- **laser_engine** — owns the point ring and the producer API
  (`laser_engine_point` / `laser_engine_points`). A `gptimer` auto-reload alarm
  fires at a fixed sample rate (ILDA-style, default 30 kpps); the ISR pops one
  point per tick, maps its ILDA coordinates to galvo DAC codes and its 8-bit
  color to the laser DAC, and writes them via `isr_spi`. Blanks the laser on
  underrun for safety.
- **point_stream** — triple-buffered DRAM scene store plus a TCP receiver
  (port 7777) that parses standard ILDA off the socket. The renderer loops the
  latest published scene.
- **artnet_control** — an Art-Net (DMX-over-UDP, port 6454) receiver that maps a
  15-channel DMX fixture onto the device's control state: a channel selects the
  render mode (built-in Lissajous **pattern** vs the streamed **scene**), the rest
  tune the pattern's frequency, size, hue, intensity, and morph plus the
  "aliasing" effects — a sliding blank gap, color-as-a-function-of-`t`, and
  per-axis frequency morphing. Drive it from a console, from QLC+/Resolume, or
  from the [`artnetctl`](tools/artnetctl) CLI. Also answers Art-Net **ArtPoll** for
  discovery, so it shows up in any controller's node list (and `artnetctl
  --discover`).

## Control (pattern vs. stream)

The projector starts on its built-in Lissajous and switches to the streamed
scene on command — both selected at runtime over Art-Net, no reflash:

```sh
# don't know the IP? discover it (or use --host auto / shadowgraph.local)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --discover

# point at the streamed scene, then push frames
cargo run --manifest-path tools/artnetctl/Cargo.toml -- --host auto --mode stream
./play.sh --once tools/svg2scene/examples/chicken.svg

# back to a live pattern (5:4 Lissajous, half size, slow morph)
cargo run --manifest-path tools/artnetctl/Cargo.toml -- \
    --host shadowgraph.local --mode pattern --fx 5 --fy 4 --size 0.5 --morph 0.2
```

The device is reachable three ways without hunting for its DHCP address:
`artnetctl --discover` / `--host auto` (Art-Net **ArtPoll**), or `shadowgraph.local`
(**mDNS** — also works as `--host` for `ildaplay`). See
[`tools/artnetctl`](tools/artnetctl) for the full option list and the DMX channel
map.

## Host unit tests

`point_ring` is pure C and unit-tested on the host with googletest via a
standalone CMake project under `host_test/`:

```sh
# from the repo root
cmake -S components/point_ring/host_test -B components/point_ring/host_test/build
cmake --build components/point_ring/host_test/build
ctest --test-dir components/point_ring/host_test/build --output-on-failure
```

This requires the googletest submodule fetched during setup. `point_stream` and
`artnet_control` carry their own `host_test/` projects too (the Art-Net parser
and DMX decode, the ILDA record decode and triple buffer) — build them the same
way, swapping in the component path.
