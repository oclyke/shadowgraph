# shadowgraph

## Setup

Clone with submodules and install the toolchain once:

```sh
# Clone the repo
git clone https://github.com/oclyke/shadowgraph.git

cd shadowgraph

# Update the submodules
git submodule update --init ./third-party/github.com/espressif/esp-idf

# Source the environment setup script from the repo root
source env.sh

# Install the IDF
$IDF_PATH/install.sh
```

## Every new shell session

```sh
. env.sh
```

This sets `IDF_PATH` to the pinned ESP-IDF submodule and puts `idf.py` on your `PATH`.

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
