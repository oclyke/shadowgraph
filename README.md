# shadowgraph

```
cargo run --manifest-path tools/svg2scene/Cargo.toml tools/svg2scene/examples/bicycle.svg --debug-output-dir && \
cargo run --manifest-path tools/frame_send/Cargo.toml -- --host 172.20.10.2  --scene output/scene.bin`
```

Firmware for a laser projector built on the [LaserScanner](https://github.com/oclyke/LaserScanner)
hardware: an ESP32 drives two galvanometer axes and an RGB laser from a
time-ordered command stream. Renderers push `goto` / `laser` / `dwell` commands
into a lock-free queue; a hardware-timer-paced consumer task clocks them out to
the DACs.

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
│   ├── byte_queue/         # SPSC lock-free byte ring buffer
│   ├── laser_command/      # type-value command codec over byte_queue
│   ├── laser_engine/       # command queue + timer-driven DAC consumer
│   ├── wifi_sta/           # WiFi station bring-up (join an AP / hotspot)
│   ├── wifi_ap/            # WiFi SoftAP bring-up (host our own network)
│   ├── udp_echo/           # UDP echo server (host ↔ device link check)
│   ├── frame_buffer/       # variable-length frame FIFO in one arena (pure C)
│   └── frame_stream/       # TCP frame recv + UDP playout clock + pump
├── tools/
│   ├── svg2scene/          # SVG → laser scene converter (Rust)
│   ├── udp_echo/           # UDP echo client (Python / click)
│   └── frame_send/         # push one frame + advance (Rust)
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
- **byte_queue** — single-producer/single-consumer lock-free byte ring buffer.
  Multi-byte records are published atomically via a release/acquire handoff.
- **laser_command** — type-value codec for `goto` / `laser` / `dwell` commands
  on top of `byte_queue`.
- **laser_engine** — owns the command queue and the producer API
  (`laser_engine_goto/laser/dwell`). A `gptimer` paces a high-priority consumer
  task that decodes commands and writes the DACs (SPI runs in task context, not
  the ISR). Blanks the laser on underrun for safety.
- **wifi_sta / wifi_ap** — minimal WiFi bring-up. `wifi_sta` joins an existing
  access point (a phone hotspot) so the device gets an IP on the host's link;
  `wifi_ap` stands up our own SoftAP instead. Pick one in `main.c` via the
  `ENABLE_STA` / `ENABLE_AP` switches.
- **udp_echo** — a background task that binds a UDP socket and echoes every
  datagram straight back to the sender. The smallest end-to-end check that the
  host can reach the device over WiFi. See below.
- **frame_buffer** — a FIFO of variable-length frames in one continuous 96 KB
  arena (forbidden wrap, reclaim floored at the playing frame). Pure C,
  host-tested. The store behind the animation playout path.
- **frame_stream** — the networked playout path: a TCP server receives frames
  into `frame_buffer`, a UDP listener advances the playout cursor (`NEXT` ticks),
  and a pump task loops the active frame into `laser_engine`. See
  `docs/FRAME_STREAMING.md` and the demo below.

## Networking & UDP echo

The device brings up WiFi (station mode by default) and starts a UDP echo server
on port `3333`. On boot the serial log reports the address it was given and that
the echo server is up:

```
wifi_sta: got ip: 172.20.10.2
udp_echo: listening for UDP on port 3333
```

Note the IP from the `got ip:` line — it is assigned by DHCP and will not always
be the same. The host must be on the **same network** the device joined (the
SSID in `main.c`, e.g. a phone hotspot), otherwise packets never arrive even
though `sendto` reports success.

A matching host-side client lives in `tools/udp_echo/`. It sends a message,
waits for the echo, and verifies the reply matches (non-zero exit on
timeout/mismatch, so it doubles as a reachability check):

```sh
cd tools/udp_echo
pip install -r requirements.txt          # one-time: installs click

# defaults to message "Hello World" on port 3333
./udp_echo.py --host 172.20.10.2
./udp_echo.py --host 172.20.10.2 --port 3333 "ping"
```

Expected output:

```
-> 172.20.10.2:3333  'Hello World'
<- 172.20.10.2:3333  'Hello World'
echo matched
```

## Frame streaming demo

The device can loop a *frame* (a self-contained scene) streamed in over the
network, advancing on an external tick. Frame **data** arrives reliably over TCP
(port 7777); the **playout clock** — a relative `NEXT` tick — arrives over UDP
(port 7778) so it never blocks behind bulk data. `NEXT` advances the playout
cursor one frame (the host stays stateless — no ids to track); advancing past the
last received frame leaves the queue empty (the laser blanks — no wrap). Frames
are stored in a 96 KB arena and the active one loops locally, so a static image
costs zero ongoing traffic. Full contract:
`docs/FRAME_STREAMING.md`. Enabled by default (`ENABLE_FRAME_STREAM` in
`main.c`); when on, the frame pump is the laser engine producer instead of the
built-in demo renderers.

The `tools/frame_send` Rust CLI is the host side of the basic demo: it builds one
closed-loop frame, pushes it over TCP, **waits for the device's commit ack**, then
advances the device to it with a `NEXT` tick (the ack ordering keeps the tick from
racing ahead of the frame). Run it repeatedly to step an animation: each push
becomes the displayed frame. Playout is strictly FIFO — frame ids aren't used to
address frames.

```sh
cargo run --manifest-path tools/frame_send/Cargo.toml -- --host 172.20.10.2
# options: --shape square|triangle|diamond  --size COUNTS  --intensity 0..65535
cargo run --manifest-path tools/frame_send/Cargo.toml -- --host 172.20.10.2 --shape triangle
```

To render real artwork, `svg2scene` emits the same `laser_command` wire bytes, so
its output pushes straight through as a frame:

```sh
cargo run --manifest-path tools/svg2scene/Cargo.toml -- drawing.svg -o drawing.scene
cargo run --manifest-path tools/frame_send/Cargo.toml -- --host 172.20.10.2 --scene drawing.scene
```

The device log shows the frame committed and played:

```
frame_stream: frame TCP server on port 7777
frame_stream: playout-control UDP on port 7778
frame_stream: client connected from 172.20.10.x
```

As with the UDP echo, the host must be on the **same network** the device joined.

## Host unit tests

`byte_queue`, `laser_command`, and `frame_buffer` are pure C and are unit-tested
on the host with googletest. Each has a standalone CMake project under
`host_test/`:

```sh
# from the repo root (replace byte_queue with laser_command for that suite)
cmake -S components/byte_queue/host_test -B components/byte_queue/host_test/build
cmake --build components/byte_queue/host_test/build
ctest --test-dir components/byte_queue/host_test/build --output-on-failure
```

This requires the googletest submodule fetched during setup.
