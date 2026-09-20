# ARDBClient type guide for the Quest team

## What a type means

`ARDBVisualType` is an eight-bit **descriptor** attached to a registered
stream. It tells Quest what the publisher intends the application bytes to
mean. `ARDBClient` does not convert, serialize, range-check, or otherwise
validate application bytes based on that type. It sends the bytes supplied to
`print()`/`printBytes()` unchanged, followed by a CRC-16 trailer.

That distinction is important: a type is sufficient to select a Quest decoder
only when the publisher and Quest have also agreed on the field order, units,
scale, signedness, and application-byte order. Those details are not carried
by ARDB metadata.

```text
publisher: addTopic(topic, visual type, name, expected byte length)
    -> retained ARDB metadata v3 over MQTT
broker/head: validates and stores the descriptor
    -> AHSP/4 STREAM_DEFINITION(type, expected length, display name)
Quest: chooses a decoder from the type, then decodes application bytes in
       BASELINE/DELTA sample records
```

The Quest receives the **application bytes only**. It does not receive the
MQTT data-packet CRC; the head validates and removes it. Each AHSP/4 frame
which carries the definition or sample has its own separate frame CRC.

## The visual types

The values below are the complete `ARDBVisualType` enum as implemented by this
repository. “Suggested registration” is a reliable way to configure the
length check; it is not additional type validation by the client or head.

| Enum and value | Intended Quest interpretation | Suggested registration | What is actually guaranteed |
| --- | --- | --- | --- |
| `Log` (`0`) | Text for a log/debug view. | Variable: `expectedPayloadBytes = 0`. | The `char[]`, `const char*`, and `String` overloads send the supplied text bytes without a terminating NUL. ARDB does not check that the bytes are UTF-8. |
| `ScalarF32` (`1`) | One scalar floating-point measurement. Units and display precision are publisher/Quest agreement. | Fixed: `4` for the 32-bit-float schema. | The type value and the declared length are delivered. The library sends the in-memory bytes of the publisher's `float`; it does not define an application-byte order or verify IEEE-754 encoding. |
| `Vector3F32` (`2`) | Three floating-point components, normally displayed together as a vector. Component names/order and units must be agreed separately. | Fixed: `12` for three 32-bit floats. | The library will send the raw bytes of a three-element `float` array or a suitable 12-byte wire structure. It does not establish which component is x, y, or z. |
| `Imu6I16` (`3`) | Six signed 16-bit IMU values. | Fixed: `12`. | Only the enum value and byte count are defined by this repository. The component order, sensor units, scale, and application-byte order are not encoded in ARDB metadata. |
| `Imu6I16T32` (`4`) | Six signed 16-bit IMU values plus a 32-bit source timestamp. | Fixed: `16`. | The included minimal publisher example uses the order `timeMs`, `ax`, `ay`, `az`, `gx`, `gy`, `gz` in one packed 16-byte structure. That is the only in-repository example of this type; it is not enforced by `ARDBClient`. |
| `Event` (`5`) | An application-defined event record. | Variable (`0`) unless the event schema has one fixed size. | ARDB gives event bytes no special treatment. Quest needs a separately versioned schema before it can decode the payload. |
| `THREE_NUM` (`6`) | Three related numeric values. | Fixed: `12` when the agreed form is three 32-bit floats. | The TMC6300 example uses this type for three `float` PWM-duty values (`uh`, `vh`, `wh`) and sends 12 bytes. The enum itself does not require floats, their order, or their units. |
| `Binary` (`255`) | Opaque application-defined bytes. | Variable (`0`) unless the binary format is fixed. | The head forwards valid bytes to Quest unchanged. The type carries no schema, MIME type, length prefix, or encoding. |

### Consequences for a Quest decoder

- Treat `visualType` as an unsigned byte, not as proof that a payload has a
  particular layout. The broker accepts and forwards any byte value; a future
  publisher can use a value not listed above. An unknown value should be shown
  safely as an unknown/raw stream rather than decoded as one of the known
  layouts.
- A fixed `expectedPayloadBytes` is the length of the **application bytes**,
  not the MQTT payload length and not an AHSP sample-record length. For
  example, an `Imu6I16T32` stream declares `16`, while its AHSP sample record
  occupies `1` byte stream ID + `2` byte age + `1` byte length + `16` bytes
  application payload.
- Decode multibyte application fields only with a publisher-specific byte-order
  agreement. The ARDB metadata and AHSP framing use big-endian integers where
  specified, but that rule does **not** apply to application bytes. The client
  copies C++ object memory as-is. The current Pico examples therefore need an
  explicitly agreed little-endian decoder if Quest is to interpret their
  numeric fields.
- Do not infer IMU scale or units from `Imu6I16`/`Imu6I16T32`. For example,
  acceleration might be counts, g, or milli-g; gyro might be counts, degrees/s,
  or radians/s. The descriptor's display name is the only human-readable field
  carried to Quest.
- Never use a C++ structure's default layout as a wire schema. Padding can
  change `sizeof` and field offsets. Use arrays of fixed-width fields or an
  explicitly packed, size-checked wire structure.

## Publisher patterns

The following patterns match the library's typed `print()` overloads. They are
examples of payload construction, not a replacement for the missing semantic
contract (units, ordering, scale, and byte order).

```cpp
// Log: NUL is not sent.
ARDBTopic log = ardb.addTopic("a/robot/log", ARDBVisualType::Log,
                              "Robot log");
ardb.print(log, "Motor enabled");

// One scalar float.
ARDBTopic temperature = ardb.addTopic("a/robot/temp",
    ARDBVisualType::ScalarF32, "Temperature", 4);
const float temperatureC = 21.5f;
ardb.print(temperature, temperatureC);

// Three values. The team must specify that the order is x, y, z before Quest
// treats it as a vector.
ARDBTopic vector = ardb.addTopic("a/robot/vector",
    ARDBVisualType::Vector3F32, "Vector", 12);
const float xyz[3] = {1.0f, 2.0f, 3.0f};
ardb.print(vector, xyz);

// Six IMU values. This example names a proposed order explicitly; the enum
// alone does not define it.
ARDBTopic imu = ardb.addTopic("a/robot/imu", ARDBVisualType::Imu6I16,
                              "IMU", 12);
const int16_t axAyAzGxGyGz[6] = {100, 0, 1024, 0, 0, 0};
ardb.print(imu, axAyAzGxGyGz);

// The same named IMU layout used by examples/MinimalClient. Packing and the
// size assertion prevent compiler-inserted padding from changing the payload.
struct __attribute__((packed)) ImuFrame {
  uint32_t timeMs;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
static_assert(sizeof(ImuFrame) == 16, "IMU wire frame must be 16 bytes");

ARDBTopic timedImu = ardb.addTopic("a/robot/imu-t",
    ARDBVisualType::Imu6I16T32, "Timed IMU", 16);
const ImuFrame frame = {1234, 100, 0, 1024, 0, 0, 0};
ardb.print(timedImu, frame);

// The TMC6300 example's THREE_NUM convention: three float duty percentages.
ARDBTopic pwm = ardb.addTopic("a/robot/pwm", ARDBVisualType::THREE_NUM,
                              "PWM duty", 12);
const float uvw[3] = {25.0f, 50.0f, 75.0f};
ardb.print(pwm, uvw);

// Event or Binary: callers supply the explicit byte count for dynamic data.
ARDBTopic event = ardb.addTopic("a/robot/event", ARDBVisualType::Event,
                                "Event");
const uint8_t eventBytes[] = {0x01, 0x02};
ardb.printBytes(event, eventBytes, sizeof(eventBytes));
```

`print()` returns `false` without attempting a transport publish when the
handle is invalid or a fixed-size topic is given the wrong application length.
It returns `false` and increments `droppedPackets()` when it is disconnected.
There is no offline queue. A type mismatch is never detected because the
runtime data carries no type tag; only the registered descriptor has one.

## Descriptor and delivery contract

### Metadata emitted by `ARDBClient`

On MQTT connection, and before a registered topic's first data frame when its
metadata has not yet been sent, the client publishes a descriptor. The message
is retained by default and the first registered stream uses the MQTT topic
`ardb/meta/<client-id>/1` (then `/2`, `/3`, and so on). Its payload is ARDB
metadata protocol version 3:

```text
0..3    ASCII "ARDB"
4       version = 3
5       visual type (the enum value above)
6..7    expected application-payload bytes, unsigned big-endian u16
8       data-topic byte length, unsigned u8
9       display-name byte length, unsigned u8
10..    data-topic bytes, then display-name bytes
last 2  CRC-16/CCITT-FALSE of every preceding metadata byte, big-endian u16
```

An expected length of zero means variable length. Client registration rejects
a fixed length greater than 256 bytes. It also rejects an empty topic/name,
topic/name longer than 255 bytes, a conflicting duplicate topic, and a topic
count beyond `ARDB_MAX_TOPICS` (8 by default).

For this head implementation, keep the client metadata prefix at its default
`ardb/meta`: the broker recognizes only MQTT topics beginning with
`ardb/meta/`. The head further accepts only a topic up to 64 bytes, a display
name up to 48 bytes, an expected length up to 256 bytes, and its configured
stream capacity (8 by default). A descriptor accepted by the client can
therefore still be rejected by the head if it exceeds those tighter limits.

The `ARDBTopic::id()` and the final metadata-topic component are publisher
registration positions. They are not the identifier Quest should render
against. The head assigns its own one-byte stream ID when it registers the
descriptor; Quest must use the stream ID from `STREAM_DEFINITION` and sample
records.

### Data path to Quest

The publisher's MQTT data payload is:

```text
application bytes | CRC-16/CCITT-FALSE:u16 big-endian
```

The head requires the topic to be registered, validates that CRC, enforces the
declared fixed length when nonzero, and stores the most recent valid
application payload. It does not interpret the `visualType` value. It drops
the MQTT CRC before creating a Quest sample. The low-level
`publish(const char* topic, ...)` API does not register metadata, so the head
rejects its data unless that same MQTT topic was already registered.

Quest receives the retained stream definition through an AHSP/4
`STREAM_DEFINITION` frame body:

```text
stream ID:u8 | visual type:u8 | expected payload bytes:u16 big-endian |
display-name length:u8 | display-name bytes
```

The corresponding `BASELINE` and `DELTA` records are:

```text
stream ID:u8 | sample age:u16 big-endian | payload length:u16 big-endian |
application bytes
```

Definitions are upserts by head stream ID. At session start the head sends a
status frame, every current definition, and a baseline. If a definition
changes while Quest is connected, it sends current definitions followed by a
new baseline before later deltas. A missing sample is unavailable data, not a
numeric zero.

Delivery is latest-value telemetry, not a lossless log: the head stores one
sample per stream and deduplicates byte-identical payloads. After a baseline,
it emits changed values as deltas no more often than every 100 ms. Quest should
include source timestamps or sequence numbers inside an application schema when
it needs ordering or loss detection.

## Quest implementation checklist

1. Parse and CRC-check AHSP/4 frames independently of Fetch read boundaries.
2. Upsert a stream definition by its head-supplied stream ID; retain its type,
   expected length, and display name.
3. For each baseline/delta record, require that the record's payload length is
   at most 256 and matches a nonzero expected length before decoding.
4. Select a decoder from `visualType`, but make each numeric decoder depend on
   an agreed application schema. Use `DataView` with the schema's explicit
   endianness rather than relying on a JavaScript typed-array default.
5. Handle unknown types, malformed payloads, absent samples, and restarted
   stream registries without treating them as measurements.
6. Keep a per-publisher schema document for fields whose type alone is not
   enough: all IMU streams, `THREE_NUM`, `Event`, `Binary`, and any units or
   scaling applied to scalar/vector types.

## Source basis

This guide is based on the current implementation in:

- `ARDBClient/src/ARDBClient.h` and `ARDBClient/src/ARDBClient.cpp` for the
  enum, registration, metadata, and publish behavior.
- `ARDBClient/examples/MinimalClient/MinimalClient.ino` and
  `ARDBClient/examples/TMC6300/TMC6300.ino` for the timestamped IMU and
  `THREE_NUM` example layouts.
- `ARDBBroker/src/HeadTelemetry.cpp`, `ARDBBroker/src/HeadStreamSession.cpp`,
  and `ARDBBroker/src/HeadProtocol.cpp` for head validation and the Quest
  stream-definition/sample delivery path.

If the team wants a type to have a mandatory layout rather than a convention,
add that layout (including field order, units, scale, and application byte
order) to a versioned protocol specification and make the publisher and Quest
decoder change together.
