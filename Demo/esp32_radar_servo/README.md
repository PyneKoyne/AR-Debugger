# ESP32 servo radar -> ARDB

A servo pans an HC-SR04 across a 180-degree arc. Each step publishes **one
8-byte packet on one stream**: bearing and range, nothing else.

Status: compiles clean for `esp32:esp32:esp32` (core 3.3.6) with
`--warnings all`. Not yet run against hardware or a live head.

## Wiring

| Signal | GPIO | Notes |
| --- | --- | --- |
| Servo PWM | 32 | 50 Hz via LEDC, no servo library |
| HC-SR04 TRIG | 33 | 3.3 V output is enough to trigger the module |
| HC-SR04 ECHO | 25 | **5 V on a plain HC-SR04 — see below** |

**ECHO is 5 V and the ESP32 is not 5 V tolerant.** Use a 3.3 V part
(HC-SR04P / RCWL-1601), or divide: 1 kΩ from ECHO to GPIO 25, 2 kΩ from
GPIO 25 to GND. Power the servo from its own 5 V supply with a common ground —
running it off the dev board's regulator browns out the ESP32 mid-sweep.

## Build

Needs `ArduinoMqttClient` and `ARDBClient` (copy `ARDBClient/` into your Arduino
libraries folder). Fill in `arduino_secrets.h` first.

```
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX .
```

`ENABLE_ARDB 0` at the top of the sketch drops the ARDB client and prints the
sweep to Serial — the fastest way to tell a wiring fault from a broker fault.
It does **not** currently skip Wi-Fi: the sketch still includes
`arduino_secrets.h` and still calls `wifiStart()`/`netSupervisor()`. Making the
Wi-Fi path conditional too is an open item below.

## Polar form

The servo's own 0–180 travel *is* the bearing, so no extra calibration step is
needed: theta is measured counter-clockwise from the sensor's right-hand axis
in the horizontal plane.

```
        theta = 90  (forward)
               |
               |
  theta = 180 --+-- theta = 0
   (left)       ^        (right)
             sensor
```

A sample is the pair `(theta, r)`: bearing in degrees, range in centimetres
from the sensor face. `r` is `echo_us * 0.0343 / 2`, and the sketch reports a
sample as invalid rather than guessing when the echo times out or falls outside
`MIN_RANGE_CM`/`MAX_RANGE_CM`.

## Wire schema

ARDB metadata carries a type byte and a byte count — **not** field order,
units, or byte order (see `ARDBClient/Quest_Type_Report.md`). Those are fixed
here and the Quest decoder must mirror them. Every float is IEEE-754 binary32
**little-endian**, which is the ESP32's in-memory layout copied verbatim by
`ARDBClient`.

| MQTT topic | Type | Bytes | Payload |
| --- | --- | --- | --- |
| `a/radar` | `Binary` (255) | 8 | `float[2] { thetaDeg, rCm }` |

| offset | bytes | field | meaning |
| --- | --- | --- | --- |
| 0 | 4 | `thetaDeg` | bearing, 0–180, as described above |
| 4 | 4 | `rCm` | range in cm from the sensor face; **0.0 = no echo** |

Worked example — bearing 90°, range 123.4 cm:

```
app bytes : 00 00 B4 42  CD CC F6 42     (8 bytes)
on MQTT   : <those 8 bytes> 91 8D        (+ CRC-16/CCITT-FALSE, big-endian)
```

Decoder notes, in the order they will bite:

- **`rCm == 0.0` is the no-echo sentinel, not a target at the origin.** With
  only two fields there is nowhere to put a validity flag, so zero carries it.
  Zero is safe for this because a real reading is always ≥ `MIN_RANGE_CM`. Plot
  `rCm` without testing for zero and every empty bearing draws a false contact
  on top of the sensor.
- **Every step publishes, echo or not.** That is deliberate: a packet per step
  lets the viewer clear a stale contact instead of leaving the last hit on
  screen forever.
- **`Binary` carries no schema.** The enum has no two-number type, so the
  length and this table are the entire contract. Decode the two floats as
  little-endian binary32.
- Consecutive packets always differ, because `thetaDeg` changes every step —
  which matters, since the head drops a sample byte-identical to the one it
  already holds.

## Timing

`STEP_PERIOD_MS = 100` is the smallest period that satisfies all three limits
in the chain, so every published point actually reaches the Quest:

1. HC-SR04 wants ≥ 60 ms between triggers or the previous burst is still
   ringing.
2. A 6° step plus settling is ~15–30 ms on an SG90-class servo; measuring one
   full period after the move command means it is parked.
3. The head emits deltas no more often than every 100 ms
   (`kDeltaMinIntervalMs`, `ARDBBroker/src/HeadConfig.h`). Publishing faster
   does not yield more samples on the Quest — it discards the intermediate
   ones.

At 6° / 100 ms a 0→180 pass is 30 steps = 3.0 s. Keep `SWEEP_STEP_DEG` an even
divisor of the arc or the sweep reverses before reaching the far endpoint.

`ARDBClient` is constructed at 25 Hz rather than the default 10 Hz on purpose:
its per-topic gate restarts from the moment a publish *finishes*, so a 10 Hz
limit against a 100 ms step drops a point whenever a publish takes longer than
0 ms — which is always. The head's 100 ms interval is the intended pacer.

## Open items for integration

- **Wi-Fi is not yet optional.** `ENABLE_ARDB 0` removes the ARDB client but
  not the Wi-Fi join or the `arduino_secrets.h` include, so a bench test
  without credentials still needs the file to exist. Guarding those on
  `ENABLE_ARDB` would make the radar genuinely standalone.
- `.gitignore` excludes only `Demo/esp32cam_openai_vision/arduino_secrets.h`.
  Add `Demo/esp32_radar_servo/arduino_secrets.h` before committing real
  credentials.
- 1 stream is registered now, against a cap of 8 on both the client
  (`ARDB_MAX_TOPICS`) and the head (`ARDB_HEAD_MAX_STREAMS`). The ESP32-CAM
  demo registers 4, so both publishers now fit on one head with room to spare.
- No Quest decoder exists for this layout yet.
