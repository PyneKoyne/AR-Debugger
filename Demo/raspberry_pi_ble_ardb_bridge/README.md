# Raspberry Pi BLE to ESP32 ARDB bridge

This example bypasses the Raspberry Pi's weak Wi-Fi antenna. The Pi is a BLE
GATT peripheral; an ESP32 is the BLE central and uses its own Wi-Fi connection
to publish received data through `ARDBClient` to the MQTT broker.

```text
Raspberry Pi -- BLE notifications --> ESP32 -- Wi-Fi/MQTT --> ARDB broker
```

The ESP32 publishes each complete Pi message as an `ARDBVisualType::Binary`
stream on `a/pi/ble`. It deliberately keeps the ordinary ARDB limit of 64
application bytes. The BLE framing works with the default 23-byte ATT MTU, so
the Pi fragments each message into notifications with at most 16 data bytes.
The ESP32 rejects malformed, out-of-order, and oversize messages; it never
publishes a partial message.

## Raspberry Pi setup

Install and enable BlueZ, then install the Python dependencies used by the
included GATT-peripheral script. Package names vary slightly by Raspberry Pi OS
release; a typical setup is:

```bash
sudo apt install bluetooth python3-gi
python3 -m pip install --user bluezero
sudo systemctl enable --now bluetooth
```

Run the peripheral script with the privileges needed to register a local GATT
application:

```bash
sudo python3 raspberry_pi_ble_peripheral.py
```

The script reads one message per input line. Start it, wait for `ESP32
subscribed`, then type `temperature=23.5` and press Enter to publish those
application bytes on `a/pi/ble`:

```bash
sudo python3 raspberry_pi_ble_peripheral.py
```

For a continuous source, keep the script running and write newline-delimited
binary-safe messages to its standard input, or replace `read_stdin()` with the
Pi sensor code and call `GLib.idle_add(bridge.send, payload)`.

## ESP32 setup

1. Copy `arduino_secrets.h.example` to `arduino_secrets.h` and set the Wi-Fi
   credentials and MQTT broker address reachable by the **ESP32**.
2. Install `ArduinoMqttClient` and the local `ARDBClient` library. `BLEDevice`
   is supplied by the ESP32 Arduino core.
3. Flash `esp32_ble_ardb_bridge.ino` to an ESP32 with both BLE and Wi-Fi.

The ESP32 scans for the Pi's advertised service, reconnects after a disconnect,
and subscribes to its notification characteristic. The bridge intentionally
does not enable BLE pairing or encryption; configure BlueZ pairing and a
trusted-device policy before using it outside a controlled lab.
