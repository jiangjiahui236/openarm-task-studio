# ESP32-C3 Remote Record Button

## Arduino setup

This sketch requires the ESP32 Arduino Core. The `WiFi` library bundled with
the classic Arduino IDE is for the old Arduino WiFi Shield and cannot compile
this ESP32-C3 firmware.

In Arduino IDE:

1. Open `File > Preferences` and add this Boards Manager URL:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Open `Tools > Board > Boards Manager`, search for `esp32`, and install
   **esp32 by Espressif Systems**.
3. Select `Tools > Board > esp32 > ESP32C3 Dev Module` (or the exact
   ESP32-C3 board model), then select its serial port.

The compile output must use an ESP32 core path. If it mentions
`/snap/arduino/.../libraries/WiFi`, the ESP32-C3 board/core is not selected.

Command-line setup and compile are also supported:

```bash
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32c3 esp32c3_remote_button
```

## Wiring

- Button terminal 1: ESP32-C3 `GPIO4`
- Button terminal 2: `GND`
- The firmware uses `INPUT_PULLUP`; no external pull-up resistor is required.

## Configure

Edit the Wi-Fi values in `esp32c3_remote_button.ino`:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

Task Studio advertises `_openarm-record._udp` over mDNS. The ESP32-C3 discovers
and automatically refreshes a stale computer address, so startup order and
DHCP address changes do not require another firmware upload. On computers with
multiple network adapters, it only accepts an IPv4 address on the ESP32's current
Wi-Fi subnet, avoiding unreachable Ethernet, VPN, and loopback addresses. Both devices must be on the
same LAN, and the computer's Avahi service must be running. An optional fixed
address can be placed in `TASK_STUDIO_FALLBACK_IP`; it is empty by default.

## Operation

1. Start Task Studio and start motion teaching.
2. Select `TEACH LEFT`, `TEACH RIGHT`, or `TEACH BOTH`.
3. Press the physical button once to append the current pose as confirmed Move point(s).
4. Motion teaching continues without pausing. Use `Stop Motion Teaching` when the sequence is finished.

The firmware discovers Task Studio during boot and retries discovery every 5 seconds while unavailable. Each press uses a sequence number and waits for Task Studio's ACK; missing ACKs are retried automatically and force rediscovery after 5 seconds. Duplicate UDP packets receive ACKs but create only one task point. A one-second `PING`/`PONG` heartbeat carries the live GPIO `DOWN`/`UP` state for the Task Studio `Button Test` tab; button release is also reported immediately. Legacy firmware sending plain `RECORD` remains supported, but it cannot provide continuous connection or release state.
