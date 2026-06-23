# shadowgraph

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
│   ├── point_ring/         # SPSC lock-free ring of ILDA points
│   └── laser_engine/       # point ring + fixed-rate timer-ISR DAC consumer
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

## Host unit tests

`point_ring` is pure C and unit-tested on the host with googletest via a
standalone CMake project under `host_test/`:

```sh
# from the repo root
cmake -S components/point_ring/host_test -B components/point_ring/host_test/build
cmake --build components/point_ring/host_test/build
ctest --test-dir components/point_ring/host_test/build --output-on-failure
```

This requires the googletest submodule fetched during setup.
