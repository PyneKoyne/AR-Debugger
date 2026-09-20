# ARDBBroker: portable broker and Quest head node

ARDBBroker provides a local raw MQTT broker and a one-way HTTPS telemetry API on the same Wi-Fi development board. The Quest runs the full WebXR application from its own HTTPS origin; this firmware never hosts the WebXR page.

```text
ARDB publishers -- MQTT/TCP 1883 --> broker/head device -- AHSP/3 HTTPS/TCP 443 --> Quest WebXR app
```

## Platform support

The MQTT contract, runtime stream registry, CRC validation, latest-value cache, AHSP/3 framing, baseline/delta policy, and Quest API are shared. Only Wi-Fi and HTTPS transport are platform adapters.

| Target | Verified build profile | HTTPS adapter |
| --- | --- | --- |
| Raspberry Pi Pico 2 W | `rp2040:rp2040:rpipico2w`, Arduino-Pico 6.1.0 | Arduino-Pico BearSSL |
| ESP32 Wi-Fi boards | `esp32:esp32:esp32`, Arduino-ESP32 3.3.11 | ESP-IDF `esp_https_server` |

Portable means that these two Arduino platforms have adapters; it does not mean every microcontroller, ESP32 variant, or core version can host the HTTPS service. A target must have Wi-Fi and sufficient TLS memory.

The Pico adapter is deliberately pinned to Arduino-Pico 6.1.0. Its secure accept path misses a lwIP delayed-backlog acknowledgement, so the Pico-only `HeadSecureServer` wrapper supplies it before BearSSL accepts a connection. Do not change this wrapper or its compile-time version guard without on-device reconnect regression testing. ESP32 uses its native HTTPS server and does not use the Pico workaround.

## Boilerplate and credentials

Start from [examples/Boilerplate/Boilerplate.ino](examples/Boilerplate/Boilerplate.ino). It contains placeholders for every deployment-specific value: Wi-Fi mode, SSID, passphrase, certificate PEM, and private-key PEM. No library build source reads credentials or a private key.

Copy the boilerplate directory outside this repository (or make a private sketch from it) before inserting credentials. Never commit a sketch that contains a Wi-Fi passphrase or private key.

If this checkout contains legacy ignored `HeadCredentials.h` or `BrokerNetworkCredentials.h` files, they are no longer referenced. Preserve or securely remove them according to your local secret-handling policy; do not copy them into the library.

The sketch creates one `ardb_broker::Runtime` and calls `ardb.update()` from
`loop()`. `Runtime` owns the broker, bounded dynamic telemetry registry, and HTTPS service;
the sketch owns deployment policy. Select `Mode::AccessPoint` or
`Mode::Station` in `kNetwork`, fill its two Wi-Fi strings, and paste the two
PEM values. The AP address field is ignored in station mode.

### Access-point mode

`Mode::AccessPoint` creates the isolated development network. Its defaults from `BrokerNetworkConfig.h` are:

| Service | Value |
| --- | --- |
| Device/gateway | `192.168.4.1` |
| MQTT | `mqtt://192.168.4.1:1883` |
| Health | `https://192.168.4.1/api/v2/health` |
| Live stream | `https://192.168.4.1/api/v2/live` |

Connect both Quest and publishers to this AP. If its address changes, create a certificate with a matching IP SAN.

### Existing-network mode

`Mode::Station` joins the supplied Wi-Fi network using DHCP, waits for association, then starts MQTT and HTTPS. It prints the assigned address and complete service URLs to serial.

For repeatable Quest integration, reserve the device's DHCP address and issue the certificate for it, or supply LAN DNS and certificate that name. This firmware does not configure static station addressing or mDNS. Do not use `192.168.4.1` in station mode unless your network really assigned it.

AP+station concurrency is intentionally excluded. One network identity avoids ambiguous broker URLs and unnecessary radio/RAM use.

## TLS

Replace the PEM placeholders in the boilerplate sketch. Use an ECDSA P-256 certificate whose IP SAN or DNS SAN is the exact host Quest will fetch.

For the default AP address:

```bash
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
  -keyout key.pem -out cert.pem -days 365 -subj '/CN=192.168.4.1' \
  -addext 'subjectAltName=IP:192.168.4.1'
```

Paste the PEM contents into the boilerplate sketch. For a self-signed development certificate, open the health URL in the Quest browser and explicitly accept it before opening the WebXR application. `curl -k` is a local diagnostic only, never a trust model.

Build with either supported profile:

```bash
arduino-cli compile --libraries . --fqbn rp2040:rp2040:rpipico2w ARDBBroker/examples/Boilerplate
arduino-cli compile --libraries . --fqbn esp32:esp32:esp32 ARDBBroker/examples/Boilerplate
```

Select the matching board and partition scheme for another ESP32 model. A successful build is not hardware validation.

## MQTT contract

TinyMqtt listens on raw MQTT 3.1.1/TCP port `1883`. It is a trusted-lab component, not a production security boundary.

- MQTT is unencrypted and current TinyMqtt behavior accepts clients that omit credentials.
- There are no ACLs, quotas, persistence guarantees, QoS 1 recovery guarantees, or topic isolation.
- Retained-message capacity defaults to `ARDB_HEAD_MAX_STREAMS`, so one retained descriptor fits per accepted stream. If a custom `Runtime` constructor capacity is used, it must be at least the number of descriptors expected to survive a broker restart. Ordinary data publishes should not be retained.
- TinyMqtt and the optional event logger use dynamic containers; keep the lab client population small.

The Quest-facing registry is populated from retained metadata published to
`ardb/meta/<client-id>/<topic-id>`. Each descriptor supplies its MQTT topic,
visual type, declared application-byte length, and display name. The broker
assigns an 8-bit stream ID in registration order and subscribes with one local
`#` wildcard, so it can accept data immediately after the descriptor.

Set `ARDB_HEAD_MAX_STREAMS` before including `ARDBBroker.h` to choose capacity:

```cpp
#define ARDB_HEAD_MAX_STREAMS 8
#include <ARDBBroker.h>
```

Every stream slot, topic string, display name, and 64-byte latest-value cache
is statically reserved. A descriptor that exceeds the configured count, topic
limit (64 bytes), name limit (48 bytes), or payload limit (64 bytes) is
rejected. This is dynamic registration, not heap allocation. Stream IDs are
stable until the broker restarts; after a restart, retained descriptors must be
replayed before their data can be shown.

Set a publisher's `ARDB_MAX_TOPICS` no higher than the broker capacity, or
expect excess descriptors to be rejected. Retained metadata consumes one
broker retention slot per descriptor.

ARDB metadata protocol v3 is required. Its body is:

```text
"ARDB" | version:3:u8 | visual type:u8 | expected payload bytes:u16 BE |
topic length:u8 | display-name length:u8 | topic bytes | name bytes |
CRC-16/CCITT-FALSE:u16 BE
```

An expected payload size of zero permits variable payloads up to 64 bytes.
The declared type, byte length, field order, units, scale, signedness, and byte
order remain the publisher/Quest contract.

Every ARDB data publish is:

```text
application bytes | CRC-16/CCITT-FALSE:u16 big-endian
```

CRC parameters are polynomial `0x1021`, initial `0xFFFF`, no reflection, and no final XOR. The checksum covers application bytes only. The head removes the CRC, rejects bad or short data as `malformed`, rejects unregistered topics or wrong lengths as `rejected`, and stores one valid application value per registered stream.

## HTTPS API

| Request | Result |
| --- | --- |
| `GET /api/v2/health` | Short JSON diagnostic response, then close |
| `GET /api/v2/live` | Long-lived chunked `application/octet-stream` AHSP/3 response |

Use simple GET requests: no body, custom headers, credentials, or query string. The header budget is 384 bytes and Pico uses a three-second request deadline. Exactly one live stream is supported. The Quest must own one reader, close it before another opens, and rely on `STATUS` rather than concurrent health checks.

The health response is:

```json
{"protocol":3,"streams":0,"accepted":0,"deduplicated":0,"malformed":0,"rejected":0}
```

Development responses include `Access-Control-Allow-Origin: *`, `Cache-Control: no-store`, and `X-Content-Type-Options: nosniff`. Before distribution, replace the wildcard with the exact Quest origin and design real TLS trust, MQTT security, and authorization. CORS is not authorization.

## AHSP/3

Fetch read boundaries are arbitrary. A Quest parser must accumulate bytes and parse frames itself; it must not use `ReadableStream` reads or HTTP chunks as frame boundaries.

All multibyte integers are unsigned big-endian:

```text
0..1   magic = 0xA7DB
2      protocol version = 3
3      frame type
4..7   frame sequence:u32
8..11  device serialization time from millis():u32
12..13 body length:u16
14..   body
last2  CRC-16/CCITT-FALSE of every preceding byte:u16
```

Validate magic, version, declared body length, and CRC before applying a frame.
The session begins with `STATUS`, each currently registered
`STREAM_DEFINITION` in stream-ID order, then a `BASELINE`. When a descriptor is
added or changes while the Quest is connected, the broker repeats all current
definitions and sends a new complete baseline before sending later deltas. The
Quest must treat a definition as an upsert by stream ID.

| Value | Type | Frequency |
| ---: | --- | --- |
| `0x01` | `STATUS` | At start and every five seconds |
| `0x02` | `STREAM_DEFINITION` | At start and after a registry change |
| `0x03` | `BASELINE` | After initial or replacement definitions |
| `0x04` | `DELTA` | Changed data, no more often than every 100 ms |

`STATUS` body:

```text
valid streams:u8 | accepted:u32 | deduplicated:u32 | malformed:u32 | rejected:u32 | generation:u32
```

`STREAM_DEFINITION` body:

```text
stream ID:u8 | visual type:u8 | expected payload bytes:u16 | name length:u8 | UTF-8 name bytes
```

`BASELINE` and `DELTA` body:

```text
sample count:u8 | repeated(stream ID:u8 | sample age:u16 ms | payload length:u8 | application bytes)
```

An expected payload size of zero means variable length up to 64 bytes. Sample age saturates at `65535` ms. A missing stream means unavailable, never a zero measurement.

AHSP/3 is latest-value delivery. Each registered stream has one fixed cache
slot; there is no queue, history, replay, or backfill. Valid byte-identical
values are deduplicated. Faster updates overwrite intermediate values, and a
delta contains the final changed value at most once every 100 ms. Put source
timestamps and sequence numbers inside application bytes whenever Quest needs
them. This protocol is not suitable for lossless capture or safety-critical
control.

## Quest reconnection and verification

On Fetch failure, end of stream, invalid frame, or no valid `STATUS` for 15 seconds: abort the reader, clear rendered state, wait with bounded exponential backoff, and reconnect. Every session starts with definitions and a baseline, so no resume token exists or is needed.

After flashing, use the address printed on serial:

```bash
curl -k https://DEVICE_ADDRESS/api/v2/health
curl -k --no-buffer --max-time 3 -o /tmp/ardb-live.bin https://DEVICE_ADDRESS/api/v2/live
xxd -g 1 -l 256 /tmp/ardb-live.bin
```

`curl` decodes HTTP chunking, so the captured body starts with `A7 DB`. Repeat an interrupted live capture plus a health request at least ten times. On Pico this validates the secure-listener fix. On ESP32, repeat on the intended board and partition configuration; a generic ESP32 compilation is not hardware reconnect or memory validation.

## Source map

| File group | Responsibility |
| --- | --- |
| `examples/Boilerplate/Boilerplate.ino` | Private deployment configuration and minimal application sketch |
| `src/ARDBBroker*` | Public `Runtime` API, service startup, cooperative loop |
| `src/BrokerNetwork*` | AP/station startup and address reporting |
| `src/HeadConfig.h` | Dynamic-registry capacity, limits, timing, CORS, paths |
| `src/HeadTelemetry*` | Metadata registration, wildcard MQTT subscription, validation, cache, counters, ESP32 locking |
| `src/HeadByteWriter.h`, `src/HeadProtocol*`, `src/HeadStreamSession*` | Shared transport-neutral AHSP/3 core |
| `src/HeadHttpsStreamPico.h`, `src/HeadHttpsStream.cpp`, `src/HeadSecureServer*` | Pico HTTPS adapter and backlog fix |
| `src/HeadHttpsStreamEsp32*` | ESP32 native HTTPS adapter |

Changing frame fields, metadata fields, payload byte order, delivery semantics, TLS/CORS policy, or the one-live-client rule is a breaking integration decision. Coordinate it with publisher and Quest teams, version the protocol when needed, and rerun builds, reconnect tests, and hardware tests on every supported target.
