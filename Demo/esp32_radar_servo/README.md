# ESP32 servo radar -> ARDB

A **360° positional servo** pans an HC-SR04 back and forth across an
arc. Readings are batched and published to the ARDB head on one stream as a run
of `(bearing, range)` pairs.

Status: compiles clean for `esp32:esp32:esp32` (core 3.3.6) with
`--warnings all`, in all four configurations (normal, `ENABLE_ARDB 0`,
`SERVO_CALIBRATE 1`). Not yet run against hardware or a
live head.

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

## Servo type — check this first

This sketch drives a **positional** servo: pulse width commands an angle and
the servo holds it.

`SERVO_RANGE_DEG` must match the servo's full mechanical travel across the
500–2500 µs pulse band. A "360 servo" means `360.0f`; an ordinary hobby servo
means `180.0f`. Get it wrong and the sweep is the wrong size — 360 set on a
180° part sweeps twice as wide and hits the stops; 180 set on a 360° part
sweeps half as wide.

`SERVO_CALIBRATE 1` alternates the two arc endpoints every 2 s so you can check
the travel matches what you expect, then set the constant and flash mode 0.

> If your servo is genuinely **continuous-rotation** (it spins while a pulse is
> applied and cannot hold a position), this sketch is the wrong shape for it —
> that part needs speed-and-time dead reckoning instead.

## Build

Needs `ArduinoMqttClient` and `ARDBClient` (copy `ARDBClient/` into your Arduino
libraries folder). Fill in `arduino_secrets.h` first.

```
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload  --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-XXXX .
```

`ENABLE_ARDB 0` drops the ARDB client and prints the sweep to Serial — the
fastest way to tell a wiring fault from a broker fault. It does **not**
currently skip Wi-Fi; see Open items.

## Polar form

Theta is measured counter-clockwise from the sensor's right-hand axis in the
horizontal plane.

```
        theta = 90  (forward)
               |
               |
  theta = 180 --+-- theta = 0
   (left)       ^        (right)
             sensor
```

A reading is the pair `(theta, r)`: bearing in degrees, range in centimetres
from the sensor face. `r` is `echo_us * 0.0343 / 2`, and the sketch reports a
reading as empty rather than guessing when the echo times out or falls outside
`MIN_RANGE_CM`/`MAX_RANGE_CM`.

## Wire schema

ARDB metadata carries a type byte and a byte count — **not** field order,
units, or byte order (see `ARDBClient/Quest_Type_Report.md`). Those are fixed
here and the Quest decoder must mirror them.

| MQTT topic | Type | Bytes | Payload |
| --- | --- | --- | --- |
| `a/radar` | `Binary` (255) | 8–64, multiple of 8 | `N × float[2] { thetaDeg, rCm }` |

Each packet is a **run of consecutive readings, oldest first**.
`N = payloadLength / 8`, `1 ≤ N ≤ 8`. All floats are IEEE-754 binary32
**little-endian**.

| offset | bytes | field | meaning |
| --- | --- | --- | --- |
| 8·i + 0 | 4 | `thetaDeg` | bearing of reading *i*, 0–180 |
| 8·i + 4 | 4 | `rCm` | range in cm; **0.0 = no echo** |

Worked example — one reading at bearing 90°, range 123.4 cm:

```
app bytes : 00 00 B4 42  CD CC F6 42     (8 bytes, N = 1)
on MQTT   : <those 8 bytes> 91 8D        (+ CRC-16/CCITT-FALSE, big-endian)
```

Decoder notes, in the order they will bite:

- **`rCm == 0.0` is the no-echo sentinel, not a target at the origin.** With
  only two fields there is nowhere to put a validity flag, so zero carries it.
  Zero is safe because a real reading is always ≥ `MIN_RANGE_CM`. Plot `rCm`
  without testing for zero and every empty bearing draws a false contact on top
  of the sensor.
- **Read `N` from the payload length**, and reject a length that is not a
  multiple of 8. The stream is registered variable-length, so the head does not
  check this for you.
- **A single-reading packet is byte-identical to the old one-reading format**,
  so a decoder written for that still works — it just sees `N = 1`.
- Every ping publishes, echo or not, so the viewer can clear a stale contact
  instead of leaving the last hit on screen forever.

## Motion, resolution and drift

The sweep runs in its own FreeRTOS task pinned to **core 0**; `loop()` owns
core 1 and everything that can block on the network.

Between reversals the servo is **not re-commanded at all** — LEDC holds the
pulse, so the horn turns at a constant speed and the motion is genuinely
continuous rather than a fast sequence of steps. `servoRun()` is called only at
a reversal.

### Ping spacing

Spacing is what makes the plot look continuous or sparse:

```
spacing° = SERVO_DPS × PING_PERIOD_MS / 1000
```

| speed | spacing | per pass |
| --- | --- | --- |
| 90 °/s | 5.4° | 2.0 s |
| 60 °/s | 3.6° | 3.0 s |
| 45 °/s | 2.7° | 4.0 s |

Set it with `SWEEP_SPEED_DPS`.

`PING_PERIOD_MS = 60` is already at the sensor's floor: the HC-SR04 datasheet
asks for ≥ 60 ms between triggers so the previous burst has stopped ringing,
and below that it starts answering with the old echo. So the only honest way to
tighten spacing further is to lower `SWEEP_SPEED_DPS`, trading seconds per
pass for degrees per reading.

**Batching is what makes the rate usable.** The head emits deltas no more often
than every 100 ms, so a one-reading packet caps the Quest at 10 readings/s no
matter how fast the sensor runs. Packing every reading taken since the last
publish into one packet delivers all of them — measured 16.6/s at
`PING_PERIOD_MS 60`, against 10/s before — through that same 10 Hz channel.
Typical batch is 1–2 readings; the 8-reading cap is 64 bytes, exactly the
head's ordinary payload limit.

### Drift

With a positional servo the horn is commanded to an absolute angle every tick,
so there is no dead reckoning and nothing to accumulate — the earlier drift
came from treating it as continuous-rotation. Position is integrated from
elapsed time rather than a fixed step so the speed stays honest when a tick
runs late, and a late tick is clamped to 4× nominal so it cannot become one
visible lurch.

### Why the sweep runs on its own core

With no broker reachable, `ARDBClient` retries every 2 s and each attempt sits
inside a TCP connect whose default timeout is 3000 ms
(`WIFI_CLIENT_DEF_CONN_TIMEOUT_MS` in the ESP32 core) — `mqttConnectTimeoutMs`
only bounds the CONNACK wait, not the connect underneath it. On a single thread
that stalls the servo for seconds at a time. The sketch does both: caps the
connect at 250 ms with `setConnectionTimeout()`, **and** drives the servo from
a core the network cannot stall.

`ARDBClient` is constructed at 25 Hz rather than the default 10 Hz on purpose:
its per-topic gate restarts from the moment a publish *finishes*, so a 10 Hz
limit against a 100 ms cadence drops a packet whenever a publish takes longer
than 0 ms — which is always. The head's 100 ms interval is the intended pacer.

## Open items for integration

- **Wi-Fi is not yet optional.** `ENABLE_ARDB 0` removes the ARDB client but
  not the Wi-Fi join or the `arduino_secrets.h` include, so a bench test
  without credentials still needs the file to exist. Guarding those on
  `ENABLE_ARDB` would make the radar genuinely standalone.
- 1 stream is registered, against a cap of 8 on both the client
  (`ARDB_MAX_TOPICS`) and the head (`ARDB_HEAD_MAX_STREAMS`). The ESP32-CAM
  demo registers 4, so both publishers fit on one head with room to spare.
- No Quest decoder exists for this layout yet.
