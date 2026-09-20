# ARDBClient protocol reference

`ARDBClient` is a small ArduinoMqttClient wrapper for AR-debug streams. Register
each stream once, call `update()` frequently, and call `print()` wherever data
is produced. V1 uses one MQTT data topic per stream.

Developers choose the exact topic in `addTopic()`: a short topic reduces MQTT
overhead, but long topics are fully supported. An `ARDBTopic` is only a local
handle; it is not an MQTT topic alias. Data is published on the registered
topic string.

## Typical use

```cpp
ARDBTopic imu = ardb.addTopic("a/demo/i", ARDBVisualType::Imu6I16T32,
                              "Demo IMU", 16);
ARDBTopic temp = ardb.addTopic("a/demo/t", ARDBVisualType::ScalarF32,
                               "Temperature", 4);

void loop() {
  ardb.update();
  ardb.print(imu, readImu());       // inferred: sizeof(ImuFrame)
  ardb.print(temp, readTemp());     // inferred: sizeof(float)
}
```

Register up to `ARDB_MAX_TOPICS` streams (8 by default). A failed or conflicting
registration returns an invalid handle.

## API and connection behavior

| Call | Behavior |
| --- | --- |
| `begin()` | Starts ARDB networking and connection attempts. |
| `update()` | Advances Wi-Fi/MQTT state and publishes pending metadata. Call it from `loop()`. |
| `connected()` | Current MQTT connection flag. |
| `addTopic(topic, type, name, expectedPayloadBytes)` | Registers a data topic and its Quest-facing descriptor. Zero permits variable-size data. |
| `print(handle, value)` | Publishes a trivially-copyable object; byte count comes from `sizeof`. |
| `print(handle, array)` | Publishes all bytes of a fixed-size array. |
| `print(handle, "text")` / `print(handle, String)` | Publishes text without its terminating NUL. |
| `printBytes(handle, data, length)` | Publishes variable-size binary; this is the only case where the caller supplies a length. |
| `publish(topic, data, length)` | Direct topic publish with checksum but without ARDB metadata. |

`print()` may be called unconditionally. When disconnected it returns `false`,
drops the frame, and increments `droppedPackets()`; no offline queue is kept.
`update()` retries after `retrySeconds`. An MQTT attempt is bounded by
`mqttConnectTimeoutMs` (250 ms by default), but cannot be completely
asynchronous on every Arduino network implementation.

Network ownership is optional and explicit:

| Callbacks supplied | Responsibility |
| --- | --- |
| `beginNetwork` and `isNetworkConnected` | ARDB starts Wi-Fi association and retries it when disconnected. |
| `isNetworkConnected` only | The application/network manager owns Wi-Fi; ARDB waits for its reported link before connecting MQTT. |
| Neither | The application establishes the network before ARDB begins; ARDB assumes the supplied `Client` transport is usable. |

`enabled = false` disables all ARDB work. The concise factory below creates the
common Wi-Fi/MQTT configuration without coupling ARDBClient to a Wi-Fi library:

```cpp
ARDBConfig config = ARDBConfig::wifiMqtt(
    ssid, password, brokerHost, clientId,
    brokerPort, retrySeconds, enabled);
```

`brokerPort`, `retrySeconds`, and `enabled` are optional and default to `1883`,
`2`, and `true`. The factory borrows its string pointers; it does not copy them.

## MQTT topics and discovery

For this registration:

```cpp
ardb.addTopic("a/demo/i", ARDBVisualType::Imu6I16T32, "Demo IMU", 16);
```

data is sent to `a/demo/i`. Its retained descriptor is sent to:

```text
<metadataTopicPrefix>/<clientId>/<stream-id>
```

With default prefix `ardb/meta`, client ID `demo-pico-01`, and first registration,
that is `ardb/meta/demo-pico-01/1`. Stream IDs are one-based registration
positions, so keep registration order stable. The Quest can subscribe to `a/#`
and `ardb/meta/#`, or to any narrower combination; MQTT handles multiple
subscriptions concurrently.

Metadata is published after connection and before that stream's first data
frame. It is retained by default. `metadataResendMs = 0` means no periodic
refresh. If a stream is removed or a client ID changes, clear its obsolete
retained descriptors from the broker.

## Wire contract

Every ARDB payload ends with a two-byte checksum. MQTT already reports a
message's payload length, so ordinary typed `print()` calls need no separate
length field. For a data message:

```text
application bytes | CRC-16/CCITT-FALSE (big-endian u16)
```

The application length is MQTT payload length minus two; payloads shorter than
two bytes are invalid. `printBytes()` needs a length only because a C++ pointer
does not retain the size of its backing data.

Metadata uses protocol version 3:

```text
0..3   "ARDB"
4      version = 3
5      visualization type
6..7   expected application-payload bytes (u16 big-endian; zero = variable)
8      data-topic UTF-8 byte length (u8)
9      display-name UTF-8 byte length (u8)
10..   data-topic bytes, then display-name bytes
last2  CRC-16/CCITT-FALSE (big-endian u16)
```

Topic and display-name strings are each limited to 255 bytes and have no NUL
on the wire. A receiver must verify magic, version, checksum, the declared
application length, and that the two declared string lengths plus 12 equal the
MQTT payload length before accepting metadata.

Checksum parameters are CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value
`0xFFFF`, no input/output reflection, final XOR `0x0000`. The checksum covers
all preceding payload bytes, not the MQTT topic. `123456789` produces `0x29B1`,
which is emitted as bytes `29 B1`.

To validate either message type: reject payloads below two bytes, calculate the
CRC over all but the final two bytes, decode those final bytes as big-endian,
and require an exact match. Then apply metadata-specific structural checks.

## Visualization payloads

| Enum | Value | Application bytes |
| --- | ---: | --- |
| `Log` | 0 | UTF-8 text, no terminating NUL |
| `ScalarF32` | 1 | one IEEE-754 `float` (4 bytes) |
| `Vector3F32` | 2 | three `float` values (12 bytes) |
| `Imu6I16` | 3 | six `int16_t` values (12 bytes) |
| `Imu6I16T32` | 4 | `uint32_t` timestamp + six `int16_t` values (16 bytes) |
| `Event` | 5 | application-defined bytes |
| `Jpeg` | 7 | JPEG bitstream, variable up to 2,048 bytes |
| `Boolean` | 8 | one byte: `0x00` false or `0x01` true |
| `Binary` | 255 | application-defined bytes |

The enum tells the Quest how to interpret a stream. ARDB leaves supplied data
unchanged except that `print(topic, bool)` serializes `Boolean` as `0x00` or
`0x01`. Define fixed-width fields and byte order for custom schemas, and avoid
compiler struct padding. The example IMU structure is explicitly packed and 16
bytes. Treat byte order as part of the protocol even when current targets share
little-endian layouts.

## Cost, defaults, and validation

Each data message adds exactly two ARDB bytes, plus normal MQTT framing and its
topic name. Separate streams make selective subscriptions and debugging simple;
there is no per-frame ARDB stream ID or multiplexing envelope. Data publishes
use QoS 0 and are not retained.

Defaults: MQTT port `1883`, retry `2` seconds, connect timeout `250` ms,
keepalive `30` seconds, retained metadata enabled, no periodic metadata
resend, and a 10 Hz data-publish limit per registered topic. Override the last
default directly in the `ARDBClient` declaration, for example:

```cpp
ARDBClient ardb(network, callbacks, config, /* dataPublishRateHz */ 25);
```

Pass `0` for an unlimited data rate. Metadata and connection work are not
rate-limited; rate-limited data sends return `false` and increment
`droppedPackets()`. Configuration and registered string pointers are borrowed,
not copied; keep them alive while the client exists.

The source was syntax-checked with Arduino/ArduinoMqttClient-compatible test
headers. Mock MQTT tests cover metadata ordering and IDs, periodic metadata
refresh, and the CRC vector above. Target-board builds and Quest/broker
interoperability remain to be tested on the intended hardware.
