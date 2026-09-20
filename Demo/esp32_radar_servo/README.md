# ESP32 servo radar -> ARDB

A servo pans an HC-SR04 across a 180-degree arc. Each step publishes one polar
sample to the ARDB head, plus its Cartesian projection and a per-sweep nearest
-target report.

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

Set `ENABLE_ARDB 0` at the top of the sketch to sweep and print to Serial with
no network at all — the fastest way to tell a wiring fault from a broker fault.

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
| `a/radar/polar` | `THREE_NUM` (6) | 12 | `float[3] { thetaDeg, rCm, valid }` |
| `a/radar/pt` | `Vector3F32` (2) | 12 | `float[3] { x, y, z }` in **metres** |
| `a/radar/r` | `ScalarF32` (1) | 4 | `float rCm` |
| `a/radar/near` | `THREE_NUM` (6) | 12 | `float[3] { thetaDeg, rCm, sweepIndex }` |
| `a/radar/log` | `Log` (0) | var | UTF-8 text, no NUL terminator |

Decoder notes, in the order they will bite:

- **`polar.valid` is the flag to read, not `rCm`.** An empty bearing publishes
  `rCm = 0.0` with `valid = 0.0`. Plot the radius without checking the flag and
  every empty bearing becomes a contact sitting on the sensor's own origin.
- **`a/radar/pt` is published only for valid echoes.** An absent sample means
  "nothing at that bearing", not "something at (0,0,0)". Same for `a/radar/r`.
- **`pt` is Unity-handed**: +x right, +y up, +z forward, origin at the sensor
  face. `y` is always 0 because the servo only pans. `x = r·cos θ`,
  `z = r·sin θ`, converted cm → m.
- **`near.sweepIndex` counts passes from boot** and doubles as a uniqueness
  salt: the head drops a sample byte-identical to the one it already holds, so
  without it two consecutive passes finding the same target at the same bearing
  would publish once, not twice.
- `near` is emitted once per completed pass, and skipped entirely for a pass
  with no echo.

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

- `.gitignore` excludes only `Demo/esp32cam_openai_vision/arduino_secrets.h`.
  Add `Demo/esp32_radar_servo/arduino_secrets.h` before committing real
  credentials.
- 5 streams are registered; both the client (`ARDB_MAX_TOPICS`) and the head
  (`ARDB_HEAD_MAX_STREAMS`) default to 8, and the ESP32-CAM demo registers 4.
  Running both publishers against one head exceeds that cap.
- No Quest decoder exists for `THREE_NUM` under this schema yet.
