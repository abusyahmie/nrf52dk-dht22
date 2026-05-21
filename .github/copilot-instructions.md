# Guidance for AI coding agents

Purpose: concise, actionable notes so an AI can be immediately productive in this project.

- **Big picture:** This repo contains several Zephyr / nRF Connect SDK sample apps. The `nrf52dk-dht22` app reads a DHT22 sensor via a GPIO line and exposes temperature and humidity over a custom BLE GATT service (`src/main.c`). Device configuration is done via a board overlay (`boards/nrf52dk_nrf52832.overlay`) and `prj.conf` enables Bluetooth and logging.

- **Key files:**
  - `src/main.c` — sensor sampling, bit-banged DHT22 protocol, BLE GATT service and notify logic.
  - `prj.conf` — Zephyr Kconfig settings (e.g., `CONFIG_BT`, `CONFIG_DHT`, `CONFIG_LOG`).
  - `boards/nrf52dk_nrf52832.overlay` — device-tree alias `dht22` and `dio-gpios` pin mapping.
  - `CMakeLists.txt` — build entry for the app (standard Zephyr layout).

- **Architecture & dataflow (quick):**
  - MCU GPIO bit-bangs the DHT22 in `dht22_start_signal`, `dht22_read_bit`, `dht22_read`.
  - Measurements stored as tenths: `last_temp_tenths` (int16) and `last_hum_tenths` (uint16).
  - BLE GATT service uses three custom 128-bit UUIDs: service, temp, humidity. Read handlers return 16-bit little-endian values (`sys_cpu_to_le16`).
  - Notifications triggered in main loop; characteristic attribute indices are relied upon (temp value attr index = 2, hum value attr index = 5). If you reorder `BT_GATT_SERVICE_DEFINE()` entries, update those indices.

- **Project-specific conventions** (do not assume typical defaults):
  - Values are in tenths (e.g., `last_temp_tenths` stores °C * 10). Use `sys_cpu_to_le16` for BLE payloads.
  - Device-tree alias: code uses `DT_ALIAS(dht22)`. Modify `boards/*overlay` to change the pin.
  - GATT attribute indices are hard-coded for notify calls — be cautious when editing service definition order.
  - Logging uses Zephyr `LOG_INF/LOG_ERR` and `LOG_HEXDUMP_INF` for raw sensor data.

- **Build / flash / debug (common commands)**
  - Recommended: use the nRF Connect SDK / west toolchain (this workspace targets nRF Connect SDK v2.x).
  - Build from project root for this app:
    ```bash
    cd nrf52dk-dht22
    west build -b nrf52dk_nrf52832
    ```
  - Flash to dev board:
    ```bash
    west flash
    ```
  - View serial logs (macOS): look for the USB serial device and open at 115200.
    ```bash
    # example (pick the correct device):
    screen /dev/tty.usbmodem12345 115200
    ```
  - If using RTT or Segger: use `west debug` / Segger tools as appropriate for your environment.

- **What to watch for when editing code**
  - Changing `BT_GATT_SERVICE_DEFINE()` ordering requires updating the `notify_if_enabled()` attribute indices (2 and 5 in current code).
  - `boards/*overlay` provides the `dht22` alias — update `dio-gpios` pin here instead of editing hard-coded pin numbers in `main.c`.
  - DHT22 read errors use `-1` for timing errors and `-2` for checksum mismatch; callers (main loop) currently log and skip notify.

- **Integration points & external dependencies**
  - Zephyr / nRF Connect SDK (west, CMake, Zephyr headers). The code uses these subsystems: `gpio`, `kernel`, `logging`, `bluetooth` (GATT, GAP, conn), `sys/byteorder`.
  - Board overlay ties device tree node name `dht22` to the GPIO spec consumed by `GPIO_DT_SPEC_GET`.

- **Quick examples for common edits**
  - Change sensor pin: edit `boards/nrf52dk_nrf52832.overlay` `dio-gpios = <&gpio0 11 ...>` and rebuild.
  - Add a BLE characteristic: add `BT_GATT_CHARACTERISTIC` entry inside `BT_GATT_SERVICE_DEFINE` and update any attribute-index-dependent notify logic.

- **Missing / not present**
  - There are no unit tests or CI configs in this app folder. If adding tests, prefer small, hardware-abstracted helpers for sensor logic to enable host/unit testing.

If anything in this summary is unclear or you'd like more detail (CI build commands, preferred flash/debug flow, or additional code examples), tell me which area to expand. I'll iterate quickly.
