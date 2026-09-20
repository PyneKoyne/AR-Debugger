# Pico 2 W TMC6300 BLE to ESP32 ARDB bridge

This pair of Arduino IDE sketches replaces the old direct Wi-Fi/MQTT path in
the TMC6300 example. The Pico controls the motor and supplies its PWM
telemetry over BLE; the ESP32 uses its stronger Wi-Fi connection to forward it
to the MQTT broker.

```text
Pico 2 W + TMC6300 -- BLE notification --> ESP32 -- Wi-Fi/MQTT --> ARDB broker
```

The payload is the same one used by the former TMC6300 sketch: three 32-bit
IEEE-754 floats, in order `UH`, `VH`, `WH`, each a percentage duty cycle. The
ESP32 publishes those unchanged 12 bytes as `ARDBVisualType::THREE_NUM` on
`b/demo/a`, named **Demo BLDC PWM**. Twelve bytes is within the ordinary
64-byte ARDB payload limit and within the default BLE ATT MTU.

## Flash the Pico 2 W

1. In Arduino IDE, select **Raspberry Pi Pico 2 W** and use the RP2040 Arduino
   core with its built-in `BLE` library (6.1.0 or newer).
2. Open [TMC6300.ino](../../ARDBClient/examples/TMC6300/TMC6300.ino).
3. Set the motor and TMC6300 parameters in its **USER CONFIG** section,
   connect the six PWM pins as documented in the sketch, and upload it.

The Pico advertises as `ARDB-PWM`. It does not need Wi-Fi credentials or the
ARDBClient library.

## Flash the ESP32

1. Copy `arduino_secrets.h.example` to `arduino_secrets.h` in this folder and
   fill in the Wi-Fi network and broker that the **ESP32** can reach.
2. Install `ArduinoMqttClient` and the local `ARDBClient` library in Arduino
   IDE. `BLEDevice` and Wi-Fi support come from the ESP32 Arduino core.
3. Open `esp32_pico_tmc6300_bridge.ino`, select your ESP32 board, and upload.

The ESP32 scans for the TMC6300 BLE service, subscribes to the PWM
characteristic, reconnects after a disconnect, and only forwards notifications
that are exactly 12 bytes long. It retains only the newest sample while MQTT
is unavailable, avoiding a stale telemetry backlog.

The bridge has no BLE pairing or encryption configured, so use it only in a
controlled lab unless you add an appropriate BLE security policy.
