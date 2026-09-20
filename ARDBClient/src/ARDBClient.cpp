#include "ARDBClient.h"

#include <stdio.h>
#include <string.h>

namespace {
const char* const kDefaultMetadataPrefix = "ardb/meta";
const uint8_t kMetadataProtocolVersion = 3;
const uint8_t kMetadataHeaderSize = 10;
const uint8_t kChecksumSize = 2;
}

ARDBConfig::ARDBConfig()
    : enabled(true),
      ssid(nullptr),
      password(nullptr),
      brokerHost(nullptr),
      brokerPort(1883),
      clientId(nullptr),
      retrySeconds(2),
      mqttConnectTimeoutMs(250),
      wifiConnectTimeoutMs(10000),
      keepAliveSeconds(30),
      metadataTopicPrefix(kDefaultMetadataPrefix),
      metadataRetained(true),
      metadataResendMs(0),
      metadataSpacingMs(25) {}

ARDBConfig ARDBConfig::wifiMqtt(const char* ssid,
                                const char* password,
                                const char* brokerHost,
                                const char* clientId,
                                uint16_t brokerPort,
                                uint16_t retrySeconds,
                                bool enabled) {
  ARDBConfig config;
  config.enabled = enabled;
  config.ssid = ssid;
  config.password = password;
  config.brokerHost = brokerHost;
  config.brokerPort = brokerPort;
  config.clientId = clientId;
  config.retrySeconds = retrySeconds;
  return config;
}

ARDBTopic::ARDBTopic() : _owner(nullptr), _index(0xff) {}

ARDBTopic::ARDBTopic(const ARDBClient* owner, uint8_t index)
    : _owner(owner), _index(index) {}

bool ARDBTopic::valid() const {
  return _owner != nullptr && _index != 0xff;
}

uint8_t ARDBTopic::id() const {
  return valid() ? static_cast<uint8_t>(_index + 1) : 0;
}

size_t ARDBClient::maxPayloadBytesFor(ARDBVisualType type) {
  return type == ARDBVisualType::Jpeg
             ? ARDB_MAX_JPEG_APPLICATION_PAYLOAD_BYTES
             : ARDB_MAX_APPLICATION_PAYLOAD_BYTES;
}

ARDBClient::ARDBClient(Client& network, const ARDBConfig& config,
                       uint16_t dataPublishRateHz)
    : ARDBClient(network, ARDBNetworkCallbacks(), config, dataPublishRateHz) {}

ARDBClient::ARDBClient(Client& network,
                       const ARDBNetworkCallbacks& networkCallbacks,
                       const ARDBConfig& config,
                       uint16_t dataPublishRateHz)
    : _mqtt(network),
      _networkCallbacks(networkCallbacks),
      _config(config),
      _topics{},
      _metadataSent{},
      _topicCount(0),
      _metadataIndex(0),
      _begun(false),
      _connected(false),
      _state(ARDBConnectionState::Disabled),
      _nextAttemptAt(0),
      _networkAttemptAt(0),
      _nextMetadataAt(0),
      _topicNextPublishAt{},
      _directNextPublishAt(0),
      _dataPublishIntervalMs(dataPublishRateHz == 0
                                 ? 0
                                 : (1000UL + dataPublishRateHz - 1) /
                                       dataPublishRateHz),
      _droppedPackets(0),
      _sentPackets(0),
      _lastConnectError(MQTT_CONNECTION_REFUSED) {}

void ARDBClient::begin() {
  _begun = true;
  _connected = false;
  _metadataIndex = 0;
  memset(_topicNextPublishAt, 0, sizeof(_topicNextPublishAt));
  _directNextPublishAt = 0;

  _mqtt.setCleanSession(true);
  _mqtt.setKeepAliveInterval(static_cast<unsigned long>(_config.keepAliveSeconds) * 1000UL);
  _mqtt.setConnectionTimeout(_config.mqttConnectTimeoutMs);

  if (hasText(_config.clientId)) {
    _mqtt.setId(_config.clientId);
  }

  if (!_config.enabled) {
    _state = ARDBConnectionState::Disabled;
    return;
  }

  _nextAttemptAt = millis();
  _state = ARDBConnectionState::WaitingToRetry;
}

void ARDBClient::update() {
  if (!_begun) {
    begin();
  }

  if (!_config.enabled) {
    return;
  }

  const uint32_t now = millis();

  if (_connected) {
    _mqtt.poll();

    if (!_mqtt.connected() || !networkConnected()) {
      markDisconnected(now);
      return;
    }

    if (_topicCount > 0 && _metadataIndex >= _topicCount &&
        _config.metadataResendMs != 0 && isDue(now, _nextMetadataAt)) {
      queueMetadata(now);
    }

    if (_metadataIndex < _topicCount && isDue(now, _nextMetadataAt)) {
      publishNextMetadata(now);
    }
    return;
  }

  if (_state == ARDBConnectionState::WaitingForNetwork) {
    if (networkConnected()) {
      _state = ARDBConnectionState::WaitingToRetry;
      _nextAttemptAt = now;
    } else if ((now - _networkAttemptAt) >= _config.wifiConnectTimeoutMs) {
      scheduleRetry(now);
    }
    return;
  }

  if (_state == ARDBConnectionState::WaitingToRetry && isDue(now, _nextAttemptAt)) {
    if (!networkConnected()) {
      startNetwork(now);
      return;
    }
    connectMqtt(now);
  }
}

void ARDBClient::setEnabled(bool enabled) {
  _config.enabled = enabled;

  if (!enabled) {
    _mqtt.stop();
    _connected = false;
    _state = ARDBConnectionState::Disabled;
    return;
  }

  if (_begun) {
    _nextAttemptAt = millis();
    _state = ARDBConnectionState::WaitingToRetry;
  }
}

bool ARDBClient::enabled() const {
  return _config.enabled;
}

bool ARDBClient::connected() const {
  return _connected;
}

ARDBConnectionState ARDBClient::state() const {
  return _state;
}

ARDBTopic ARDBClient::addTopic(const char* topic, ARDBVisualType type,
                               const char* name,
                               uint16_t expectedPayloadBytes) {
  if (!hasText(topic) || !hasText(name) || strlen(topic) > 255 ||
      strlen(name) > 255 ||
      expectedPayloadBytes > maxPayloadBytesFor(type) ||
      (type == ARDBVisualType::Boolean && expectedPayloadBytes != 1)) {
    return ARDBTopic();
  }

  for (uint8_t index = 0; index < _topicCount; ++index) {
    if (strcmp(_topics[index].topic, topic) == 0) {
      if (_topics[index].type == type &&
          _topics[index].expectedPayloadBytes == expectedPayloadBytes &&
          strcmp(_topics[index].name, name) == 0) {
        return ARDBTopic(this, index);
      }
      return ARDBTopic();
    }
  }

  if (_topicCount >= ARDB_MAX_TOPICS) {
    return ARDBTopic();
  }

  const uint8_t index = _topicCount;
  _topics[_topicCount++] = {topic, type, name, expectedPayloadBytes};

  if (_connected) {
    queueMetadata(millis());
  }

  return ARDBTopic(this, index);
}

bool ARDBClient::print(const ARDBTopic& topic, bool value) {
  const TopicSlot* const descriptor = topicFor(topic);
  if (descriptor == nullptr || descriptor->type != ARDBVisualType::Boolean) {
    return false;
  }

  const uint8_t encoded = value ? 1U : 0U;
  return publish(topic, &encoded, sizeof(encoded));
}

bool ARDBClient::print(const ARDBTopic& topic, const char* text) {
  return printText(topic, text, text == nullptr ? 0 : strlen(text));
}

bool ARDBClient::print(const ARDBTopic& topic, const String& text) {
  return printText(topic, text.c_str(), text.length());
}

bool ARDBClient::printText(const ARDBTopic& topic, const char* text, size_t length) {
  if (text == nullptr && length != 0) {
    return false;
  }
  return publish(topic, text, length);
}

bool ARDBClient::publish(const ARDBTopic& topic, const void* data, size_t length) {
  const TopicSlot* const descriptor = topicFor(topic);
  if (descriptor == nullptr) {
    return false;
  }
  if (descriptor->expectedPayloadBytes != 0 &&
      length != descriptor->expectedPayloadBytes) {
    return false;
  }
  // Registered ARDB streams are forwarded by the head as a single sample
  // record, whose implementation has a bounded latest-value cache. Refuse an
  // oversize variable payload locally instead of sending a packet the head
  // must reject. Direct publish(const char*, ...) remains MQTT interop.
  if (length > maxPayloadBytesFor(descriptor->type)) {
    return false;
  }

  const uint32_t now = millis();
  if (!dataPublishDue(now, _topicNextPublishAt[topic._index])) {
    ++_droppedPackets;
    return false;
  }

  // Send this stream's retained descriptor before its first data packet of a
  // connection. MQTT preserves publish ordering from this client to the broker.
  if (_connected && _mqtt.connected() && !_metadataSent[topic._index] &&
      !ensureMetadata(topic)) {
    ++_droppedPackets;
    return false;
  }

  const bool sent = publishToTopic(descriptor->topic, data, length);
  if (sent) {
    scheduleDataPublish(_topicNextPublishAt[topic._index], millis());
  }
  return sent;
}

bool ARDBClient::publish(const char* topic, const void* data, size_t length) {
  const uint32_t now = millis();
  if (!dataPublishDue(now, _directNextPublishAt)) {
    ++_droppedPackets;
    return false;
  }

  const bool sent = publishToTopic(topic, data, length);
  if (sent) {
    scheduleDataPublish(_directNextPublishAt, millis());
  }
  return sent;
}

bool ARDBClient::publishToTopic(const char* topic, const void* data, size_t length) {
  if (!hasText(topic) || (data == nullptr && length != 0) ||
      length > static_cast<size_t>(0xffffffffUL - kChecksumSize)) {
    return false;
  }

  if (!_connected || !_mqtt.connected()) {
    ++_droppedPackets;
    return false;
  }

  const uint16_t checksum = crc16Ccitt(data, length);
  uint8_t checksumBytes[kChecksumSize];
  writeCrc16BigEndian(checksum, checksumBytes);

  const bool started = _mqtt.beginMessage(topic,
                                          static_cast<unsigned long>(length + kChecksumSize),
                                          false,
                                          0);
  const bool written = started && writeBytes(data, length);
  const bool finished = written && writeBytes(checksumBytes, sizeof(checksumBytes)) &&
                        _mqtt.endMessage();

  if (!finished) {
    ++_droppedPackets;
    markDisconnected(millis());
    return false;
  }

  ++_sentPackets;
  return true;
}

uint32_t ARDBClient::droppedPackets() const {
  return _droppedPackets;
}

uint32_t ARDBClient::sentPackets() const {
  return _sentPackets;
}

int ARDBClient::lastConnectError() const {
  return _lastConnectError;
}

uint8_t ARDBClient::topicCount() const {
  return _topicCount;
}

bool ARDBClient::isDue(uint32_t now, uint32_t target) {
  return static_cast<int32_t>(now - target) >= 0;
}

bool ARDBClient::dataPublishDue(uint32_t now, uint32_t nextPublishAt) const {
  return _dataPublishIntervalMs == 0 || isDue(now, nextPublishAt);
}

void ARDBClient::scheduleDataPublish(uint32_t& nextPublishAt, uint32_t now) {
  if (_dataPublishIntervalMs != 0) {
    nextPublishAt = now + _dataPublishIntervalMs;
  }
}

bool ARDBClient::hasText(const char* value) {
  return value != nullptr && value[0] != '\0';
}

uint16_t ARDBClient::crc16Ccitt(const void* data, size_t length) {
  return crc16CcittUpdate(0xffff, data, length);
}

uint16_t ARDBClient::crc16CcittUpdate(uint16_t crc, const void* data, size_t length) {
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(data);
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(bytes[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void ARDBClient::writeCrc16BigEndian(uint16_t crc, uint8_t output[2]) {
  output[0] = static_cast<uint8_t>(crc >> 8);
  output[1] = static_cast<uint8_t>(crc & 0xff);
}

const ARDBClient::TopicSlot* ARDBClient::topicFor(const ARDBTopic& topic) const {
  if (topic._owner != this || topic._index >= _topicCount) {
    return nullptr;
  }
  return &_topics[topic._index];
}

bool ARDBClient::networkConnected() const {
  return _networkCallbacks.isNetworkConnected == nullptr ||
         _networkCallbacks.isNetworkConnected();
}

void ARDBClient::startNetwork(uint32_t now) {
  if (_networkCallbacks.beginNetwork == nullptr || !hasText(_config.ssid)) {
    scheduleRetry(now);
    return;
  }

  _networkCallbacks.beginNetwork(_config.ssid,
                                 _config.password == nullptr ? "" : _config.password);
  _networkAttemptAt = now;
  _state = ARDBConnectionState::WaitingForNetwork;
}

void ARDBClient::connectMqtt(uint32_t now) {
  if (!hasText(_config.brokerHost) || !hasText(_config.clientId)) {
    _lastConnectError = MQTT_IDENTIFIER_REJECTED;
    scheduleRetry(now);
    return;
  }

  _state = ARDBConnectionState::Connecting;
  if (_mqtt.connect(_config.brokerHost, _config.brokerPort)) {
    _connected = true;
    _state = ARDBConnectionState::Connected;
    _lastConnectError = MQTT_SUCCESS;
    // A new MQTT session can publish its first data point immediately.
    memset(_topicNextPublishAt, 0, sizeof(_topicNextPublishAt));
    _directNextPublishAt = 0;
    queueMetadata(now);
    return;
  }

  _lastConnectError = _mqtt.connectError();
  _mqtt.stop();
  scheduleRetry(now);
}

void ARDBClient::markDisconnected(uint32_t now) {
  _mqtt.stop();
  _connected = false;
  scheduleRetry(now);
}

void ARDBClient::scheduleRetry(uint32_t now) {
  _connected = false;

  if (_config.retrySeconds == 0) {
    _state = ARDBConnectionState::Disabled;
    return;
  }

  _nextAttemptAt = now + static_cast<uint32_t>(_config.retrySeconds) * 1000UL;
  _state = ARDBConnectionState::WaitingToRetry;
}

void ARDBClient::queueMetadata(uint32_t now) {
  memset(_metadataSent, 0, sizeof(_metadataSent));
  _metadataIndex = 0;
  _nextMetadataAt = now;
}

bool ARDBClient::ensureMetadata(const ARDBTopic& topic) {
  if (topic._owner != this || topic._index >= _topicCount) {
    return false;
  }

  if (_metadataSent[topic._index]) {
    return true;
  }

  const uint8_t topicId = static_cast<uint8_t>(topic._index + 1);
  if (!publishMetadata(_topics[topic._index], topicId)) {
    return false;
  }

  _metadataSent[topic._index] = 1;
  while (_metadataIndex < _topicCount && _metadataSent[_metadataIndex]) {
    ++_metadataIndex;
  }
  return true;
}

bool ARDBClient::publishNextMetadata(uint32_t now) {
  while (_metadataIndex < _topicCount && _metadataSent[_metadataIndex]) {
    ++_metadataIndex;
  }

  if (_metadataIndex >= _topicCount) {
    _nextMetadataAt = _config.metadataResendMs == 0
                          ? 0xffffffffUL
                          : now + _config.metadataResendMs;
    return true;
  }

  const uint8_t topicId = static_cast<uint8_t>(_metadataIndex + 1);
  const bool sent = publishMetadata(_topics[_metadataIndex], topicId);
  if (!sent) {
    return false;
  }

  _metadataSent[_metadataIndex] = 1;
  ++_metadataIndex;
  if (_metadataIndex >= _topicCount) {
    _nextMetadataAt = _config.metadataResendMs == 0 ? 0xffffffffUL : now + _config.metadataResendMs;
  } else {
    _nextMetadataAt = now + _config.metadataSpacingMs;
  }
  return true;
}

bool ARDBClient::publishMetadata(const TopicSlot& topic, uint8_t topicId) {
  const char* prefix = hasText(_config.metadataTopicPrefix)
                           ? _config.metadataTopicPrefix
                           : kDefaultMetadataPrefix;
  char metadataTopic[ARDB_METADATA_TOPIC_MAX_LENGTH];
  const int topicLength = snprintf(metadataTopic,
                                   sizeof(metadataTopic),
                                   "%s/%s/%u",
                                   prefix,
                                   _config.clientId,
                                   topicId);
  if (topicLength < 0 || static_cast<size_t>(topicLength) >= sizeof(metadataTopic)) {
    return false;
  }

  const size_t dataTopicLength = strlen(topic.topic);
  const size_t nameLength = strlen(topic.name);
  if (dataTopicLength > 255 || nameLength > 255) {
    return false;
  }

  const size_t descriptorLength = kMetadataHeaderSize + dataTopicLength + nameLength;
  const uint8_t header[kMetadataHeaderSize] = {
      'A', 'R', 'D', 'B',
      kMetadataProtocolVersion,
      static_cast<uint8_t>(topic.type),
      static_cast<uint8_t>(topic.expectedPayloadBytes >> 8),
      static_cast<uint8_t>(topic.expectedPayloadBytes & 0xff),
      static_cast<uint8_t>(dataTopicLength),
      static_cast<uint8_t>(nameLength)};

  uint16_t checksum = crc16Ccitt(header, sizeof(header));
  checksum = crc16CcittUpdate(checksum, topic.topic, dataTopicLength);
  checksum = crc16CcittUpdate(checksum, topic.name, nameLength);
  uint8_t checksumBytes[kChecksumSize];
  writeCrc16BigEndian(checksum, checksumBytes);

  const bool started = _mqtt.beginMessage(metadataTopic,
                                          static_cast<unsigned long>(descriptorLength + kChecksumSize),
                                          _config.metadataRetained,
                                          0);
  const bool written = started && writeBytes(header, sizeof(header)) &&
                       writeBytes(topic.topic, dataTopicLength) &&
                       writeBytes(topic.name, nameLength);
  const bool finished = written && writeBytes(checksumBytes, sizeof(checksumBytes)) &&
                        _mqtt.endMessage();

  if (!finished) {
    markDisconnected(millis());
  }
  return finished;
}

bool ARDBClient::writeBytes(const void* data, size_t length) {
  if (length == 0) {
    return true;
  }
  return _mqtt.write(reinterpret_cast<const uint8_t*>(data), length) == length;
}
