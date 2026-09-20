# ARDBClient

`ARDBClient` is a best-effort MQTT telemetry client for embedded ARDB targets.
It uses `ArduinoMqttClient`, keeps publish payloads binary, and avoids sending
application telemetry while it is disconnected.

Every ARDB data and metadata publish ends with a CRC-16/CCITT-FALSE checksum
trailer. The complete wire-format and validation rules are in the report.

See the full [library report](ARDBClient_Report.md) for the API, wire format,
connection behavior, Quest integration, resource tradeoffs, and validation
plan.

For a Quest-focused explanation of every `ARDBVisualType`, its payload
contract, and the head-to-Quest delivery path, see the
[Quest type guide](Quest_Type_Report.md).

## Install

Copy the `ARDBClient` directory into your Arduino libraries directory, then
install **ArduinoMqttClient** from Arduino Library Manager. The included
`examples/MinimalClient` sketch is a Pico W/Pico 2W example; it only depends
on `WiFi.h`, `ArduinoMqttClient`, and this library.

`ARDBClient` works with any network class that implements Arduino's `Client`
interface. Wi-Fi is supplied with optional callbacks so that board-specific
Wi-Fi libraries do not become a dependency of ARDBClient itself.

The included example constructs `ARDBClient ardb` as an object, so application
code uses ordinary dot syntax such as `ardb.update()` and `ardb.print(...)`.

## Compact configuration

For the common Wi-Fi + MQTT case, construct the standard configuration in one
statement:

```cpp
ARDBConfig config = ARDBConfig::wifiMqtt(
    WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, "demo-pico-01");
```

The optional trailing parameters are `brokerPort` (default `1883`),
`retrySeconds` (default `2`), and `enabled` (default `true`):

```cpp
ARDBConfig config = ARDBConfig::wifiMqtt(
    WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, "demo-pico-01",
    MQTT_PORT, /* retrySeconds */ 5, /* enabled */ true);
```

This factory only fills `ARDBConfig`; it does not include or call a Wi-Fi
library. As with direct field assignment, its string arguments are borrowed and
must remain valid for the client's lifetime.

## Telemetry rate

`ARDBClient` sends data for each registered topic at most 10 times per second
by default. Override that rate in the client declaration; pass `0` for no
library-side limit:

```cpp
ARDBClient ardb(network, ardbNetwork, ardbConfig,
                /* dataPublishRateHz */ 25);
```

The limit applies only to successful data messages. MQTT connection work and
ARDB metadata continue to run as required, and direct `publish(const char*,
...)` calls share their own rate limit. A rate-limited send returns `false` and
is counted by `droppedPackets()`.

## Connection behavior

Call `begin()` once and `update()` on every pass through `loop()`.
`print()`/`publish()` checks the internal MQTT-connected flag; the application
never has to pre-check it. Choose the callback arrangement that owns Wi-Fi
appropriately:

| Callbacks supplied | Wi-Fi ownership and ARDB behavior |
| --- | --- |
| `beginNetwork` and `isNetworkConnected` | ARDBClient starts Wi-Fi association, observes its state, and retries it after its Wi-Fi timeout. |
| `isNetworkConnected` only | The application/network manager starts and restores Wi-Fi; ARDBClient waits for the reported link before trying MQTT. |
| Neither | The application must establish the network before ARDBClient begins. ARDBClient assumes the supplied `Client` transport is ready and retries MQTT itself. |

In every mode, `update()` retries MQTT at `retrySeconds` and calls
`MqttClient::poll()` while connected.

For example, when the sketch owns Wi-Fi startup, leave `beginNetwork` unset
and provide only the status callback:

```cpp
WiFi.mode(WIFI_STA);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

ardbNetwork.isNetworkConnected = ardbNetworkConnected;
static ARDBClient client(network, ardbNetwork, config);
```

In that mode, the sketch or network manager is also responsible for restoring
Wi-Fi after a loss of service.

ArduinoMqttClient's `connect()` waits for the MQTT CONNACK, so no wrapper can
make its individual connect call truly asynchronous. ARDBClient makes only one
short, scheduled attempt per `update()` and defaults that wait to 250 ms. Keep
this value small enough for the target control loop.

`retrySeconds == 0` disables automatic reconnect. Disabled or disconnected
publishes are dropped immediately and counted by `droppedPackets()`.

## Topic metadata

`addTopic(topic, type, name, expectedPayloadBytes)` returns an `ARDBTopic`
handle and registers a descriptor. `expectedPayloadBytes` is the fixed
application-byte length; use `0` for a variable payload up to the ordinary
64-byte ARDB head limit, or 2,048 bytes when the type is `Jpeg`. Oversize
registered payloads are refused by the client so they do not reach the broker
only to be rejected. On MQTT connect, and then every `metadataResendMs`,
ARDBClient sends one descriptor per topic to
`ardb/meta/<clientId>/<topic-id>` (or the configured prefix). When
`metadataRetained` is true, configure the TinyMqtt broker with enough retained
slots for the number of registered ARDB topics.

The default `metadataResendMs` is `0`: descriptors are sent on ARDB connection
and before a stream's first data packet. This is the lowest-overhead setting
when retained metadata is enabled on the broker.

The binary metadata payload is:

```text
"ARDB" | protocol-version:u8 | visualization-type:u8 |
expected-payload-bytes:u16-be | topic-length:u8 | name-length:u8 |
topic bytes | name bytes | crc16:u16-be
```

Names and topic strings are stored as pointers. Use string literals or global
constant arrays, not temporary `String` values.
