#ifndef ARDB_CLIENT_H
#define ARDB_CLIENT_H

#include <Arduino.h>
#include <Client.h>
#include <ArduinoMqttClient.h>
#include <type_traits>

#ifndef ARDB_MAX_TOPICS
#define ARDB_MAX_TOPICS 8
#endif

#ifndef ARDB_METADATA_TOPIC_MAX_LENGTH
#define ARDB_METADATA_TOPIC_MAX_LENGTH 96
#endif

#ifndef ARDB_MAX_APPLICATION_PAYLOAD_BYTES
#define ARDB_MAX_APPLICATION_PAYLOAD_BYTES 64
#endif

#ifndef ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES
#define ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES 2048
#endif

static_assert(ARDB_MAX_TOPICS < 255, "ARDB_MAX_TOPICS must fit in an 8-bit topic id");
static_assert(ARDB_MAX_APPLICATION_PAYLOAD_BYTES > 0 &&
                  ARDB_MAX_APPLICATION_PAYLOAD_BYTES <= 64,
              "ARDB_MAX_APPLICATION_PAYLOAD_BYTES cannot exceed the ordinary ARDB head limit");
static_assert(ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES > 0 &&
                  ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES <= 2048,
              "ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES cannot exceed the ARDB JPEG head limit");

// The Quest uses this value to choose how to decode a topic's binary payload.
enum class ARDBVisualType : uint8_t {
  Log = 0,
  ScalarF32 = 1,
  Vector3F32 = 2,
  Imu6I16 = 3,
  Imu6I16T32 = 4,
  Event = 5,
  THREE_NUM = 6,
  Jpeg = 7,
  Boolean = 8,
  Binary = 255
};

enum class ARDBConnectionState : uint8_t {
  Disabled,
  WaitingForNetwork,
  WaitingToRetry,
  Connecting,
  Connected
};

// Implement these callbacks with the selected board's Wi-Fi library. Supply
// only isNetworkConnected when another part of the sketch owns Wi-Fi; leave
// both empty only when the network is ready before ARDBClient begins.
typedef void (*ARDBBeginNetworkCallback)(const char* ssid, const char* password);
typedef bool (*ARDBNetworkConnectedCallback)();

struct ARDBNetworkCallbacks {
  ARDBBeginNetworkCallback beginNetwork;
  ARDBNetworkConnectedCallback isNetworkConnected;

  ARDBNetworkCallbacks(ARDBBeginNetworkCallback begin = nullptr,
                       ARDBNetworkConnectedCallback isConnected = nullptr)
      : beginNetwork(begin), isNetworkConnected(isConnected) {}
};

struct ARDBConfig {
  // Creates a configuration for the common Wi-Fi + MQTT case. The string
  // pointers are borrowed, just as when assigning the fields directly, so
  // they must remain valid while the ARDBClient exists. The defaults match
  // ARDBConfig(), allowing a short setup while preserving all overrides.
  static ARDBConfig wifiMqtt(const char* ssid,
                             const char* password,
                             const char* brokerHost,
                             const char* clientId,
                             uint16_t brokerPort = 1883,
                             uint16_t retrySeconds = 2,
                             bool enabled = true);

  // Set false to make every publish a cheap no-op and disable connection work.
  bool enabled;

  // Optional Wi-Fi credentials. The callbacks above decide how to use them.
  const char* ssid;
  const char* password;

  // SSID is not an MQTT endpoint, so the broker host is explicit.
  const char* brokerHost;
  uint16_t brokerPort;
  const char* clientId;

  // 0 disables automatic reconnect. Otherwise this is the retry interval.
  uint16_t retrySeconds;

  // Bounds the wait for the MQTT CONNACK made by ArduinoMqttClient::connect().
  // The connection attempt itself is cooperative, but this library call is not
  // truly asynchronous on every Arduino network stack.
  uint16_t mqttConnectTimeoutMs;
  uint16_t wifiConnectTimeoutMs;
  uint16_t keepAliveSeconds;

  // ARDB metadata is published to <metadataTopicPrefix>/<clientId>/<topic-id>.
  // Each message is a retained QoS-0 MQTT publish when metadataRetained is true.
  const char* metadataTopicPrefix;
  bool metadataRetained;
  // 0 sends metadata only on connection and topic registration. This is the
  // lowest-overhead setting when the broker retains metadata successfully.
  uint32_t metadataResendMs;
  uint16_t metadataSpacingMs;

  ARDBConfig();
};

class ARDBClient;

// A lightweight reference to a topic registered on one ARDBClient instance.
// It is not an MQTT topic alias: the registered MQTT topic is still used for
// data messages. The handle eliminates repeated topic strings in user code and
// lets ARDB reject handles created by another client instance.
class ARDBTopic {
 public:
  ARDBTopic();

  bool valid() const;
  uint8_t id() const;

 private:
  ARDBTopic(const ARDBClient* owner, uint8_t index);

  const ARDBClient* _owner;
  uint8_t _index;

  friend class ARDBClient;
};

class ARDBClient {
 public:
  // Data for each topic is sent at most dataPublishRateHz times per second.
  // Pass 0 to disable rate limiting. Metadata and MQTT connection traffic are
  // not rate limited.
  ARDBClient(Client& network, const ARDBConfig& config,
             uint16_t dataPublishRateHz = 10);
  ARDBClient(Client& network,
             const ARDBNetworkCallbacks& networkCallbacks,
             const ARDBConfig& config,
             uint16_t dataPublishRateHz = 10);

  // Call once from setup(), after constructing the object.
  void begin();

  // Call frequently from loop(). Drives Wi-Fi/MQTT state and metadata sends.
  void update();

  void setEnabled(bool enabled);
  bool enabled() const;
  bool connected() const;
  ARDBConnectionState state() const;

  // Topic/name pointers must remain valid for the client lifetime. String
  // literals and global const character arrays are ideal; Arduino String is not.
  // A returned invalid handle means the descriptor could not be registered.
  // expectedPayloadBytes is the fixed application-byte length sent on this
  // topic. Zero permits variable payloads up to the applicable limit:
  // ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES for Jpeg, otherwise
  // ARDB_MAX_APPLICATION_PAYLOAD_BYTES.
  ARDBTopic addTopic(const char* topic, ARDBVisualType type, const char* name,
                     uint16_t expectedPayloadBytes = 0);
  ARDBTopic add_topic(const char* topic, ARDBVisualType type, const char* name,
                      uint16_t expectedPayloadBytes = 0) {
    return addTopic(topic, type, name, expectedPayloadBytes);
  }

  // Typed print overloads infer the byte length with sizeof(). Wire types must
  // be trivially copyable and have an explicitly documented byte layout.
  template <typename T>
  typename std::enable_if<!std::is_pointer<T>::value && !std::is_array<T>::value,
                          bool>::type
  print(const ARDBTopic& topic, const T& value) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "ARDB print values must be trivially copyable wire types");
    return publish(topic, &value, sizeof(T));
  }

  template <typename T, size_t N>
  bool print(const ARDBTopic& topic, const T (&values)[N]) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "ARDB print arrays must contain trivially copyable wire types");
    return publish(topic, values, sizeof(values));
  }

  template <size_t N>
  bool print(const ARDBTopic& topic, const char (&text)[N]) {
    return printText(topic, text, N == 0 ? 0 : N - 1);
  }

  // Emits a Boolean stream as exactly one byte: 0 for false, 1 for true.
  // The topic must have been registered with ARDBVisualType::Boolean.
  bool print(const ARDBTopic& topic, bool value);
  bool print(const ARDBTopic& topic, const char* text);
  bool print(const ARDBTopic& topic, const String& text);
  bool printText(const ARDBTopic& topic, const char* text, size_t length);

  // Explicit raw-binary escape hatch for dynamically sized data. Returns false
  // without touching the transport. DATA is sent without an ARDB copy or
  // serialization, followed by the protocol's two-byte CRC-16 trailer.
  bool print(const ARDBTopic& topic, const void* data, size_t length) {
    return publish(topic, data, length);
  }
  bool printBytes(const ARDBTopic& topic, const void* data, size_t length) {
    return publish(topic, data, length);
  }

  // Low-level direct MQTT-topic publishing is retained for interop. It does
  // not associate metadata with the topic; prefer a registered ARDBTopic.
  bool publish(const ARDBTopic& topic, const void* data, size_t length);
  bool publish(const char* topic, const void* data, size_t length);
  bool print(const char* topic, const void* data, size_t length) {
    return publish(topic, data, length);
  }

  uint32_t droppedPackets() const;
  uint32_t sentPackets() const;
  int lastConnectError() const;
  uint8_t topicCount() const;

 private:
  struct TopicSlot {
    const char* topic;
    ARDBVisualType type;
    const char* name;
    uint16_t expectedPayloadBytes;
  };

  static bool isDue(uint32_t now, uint32_t target);
  static bool hasText(const char* value);
  static size_t maxPayloadBytesFor(ARDBVisualType type);
  static uint16_t crc16Ccitt(const void* data, size_t length);
  static uint16_t crc16CcittUpdate(uint16_t crc, const void* data, size_t length);
  static void writeCrc16BigEndian(uint16_t crc, uint8_t output[2]);

  const TopicSlot* topicFor(const ARDBTopic& topic) const;
  bool networkConnected() const;
  void startNetwork(uint32_t now);
  void connectMqtt(uint32_t now);
  void markDisconnected(uint32_t now);
  void scheduleRetry(uint32_t now);
  void queueMetadata(uint32_t now);
  bool ensureMetadata(const ARDBTopic& topic);
  bool publishNextMetadata(uint32_t now);
  bool publishMetadata(const TopicSlot& topic, uint8_t topicId);
  bool publishToTopic(const char* topic, const void* data, size_t length);
  bool writeBytes(const void* data, size_t length);
  bool dataPublishDue(uint32_t now, uint32_t nextPublishAt) const;
  void scheduleDataPublish(uint32_t& nextPublishAt, uint32_t now);

  MqttClient _mqtt;
  ARDBNetworkCallbacks _networkCallbacks;
  ARDBConfig _config;
  TopicSlot _topics[ARDB_MAX_TOPICS];
  uint8_t _metadataSent[ARDB_MAX_TOPICS];
  uint8_t _topicCount;
  uint8_t _metadataIndex;
  bool _begun;
  bool _connected;
  ARDBConnectionState _state;
  uint32_t _nextAttemptAt;
  uint32_t _networkAttemptAt;
  uint32_t _nextMetadataAt;
  uint32_t _topicNextPublishAt[ARDB_MAX_TOPICS];
  uint32_t _directNextPublishAt;
  uint32_t _dataPublishIntervalMs;
  uint32_t _droppedPackets;
  uint32_t _sentPackets;
  int _lastConnectError;
};

#endif
