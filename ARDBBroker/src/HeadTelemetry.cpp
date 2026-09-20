#include "HeadTelemetry.h"

#include <string.h>

namespace {

constexpr char kMetadataTopicPrefix[] = "ardb/meta/";
constexpr size_t kMetadataPrefixLength = sizeof(kMetadataTopicPrefix) - 1;
constexpr uint8_t kMetadataVersion = 3;
constexpr size_t kMetadataHeaderBytes = 10;
constexpr size_t kChecksumBytes = 2;

}  // namespace

HeadTelemetry* HeadTelemetry::instance_ = nullptr;

HeadTelemetry::HeadTelemetry(MqttBroker& broker)
    : broker_(broker),
      subscriber_(&broker_, "head-telemetry"),
      streams_{},
      streamCount_(0),
      generation_(0),
      registryGeneration_(0),
      acceptedSamples_(0),
      deduplicatedSamples_(0),
      malformedSamples_(0),
      rejectedSamples_(0) {}

void HeadTelemetry::begin() {
  instance_ = this;
  subscriber_.setCallback(onMqttPublish);
  // TinyMqtt evaluates MQTT wildcards locally. A single subscription lets the
  // registry accept metadata and subsequently receive any registered topic.
  subscriber_.subscribe(Topic("#"));
}

size_t HeadTelemetry::streamCount() const {
  HeadTelemetryLockGuard guard(lock_);
  return streamCount_;
}

uint32_t HeadTelemetry::generation() const {
  HeadTelemetryLockGuard guard(lock_);
  return generation_;
}

uint32_t HeadTelemetry::registryGeneration() const {
  HeadTelemetryLockGuard guard(lock_);
  return registryGeneration_;
}

uint32_t HeadTelemetry::acceptedSamples() const {
  HeadTelemetryLockGuard guard(lock_);
  return acceptedSamples_;
}

uint32_t HeadTelemetry::deduplicatedSamples() const {
  HeadTelemetryLockGuard guard(lock_);
  return deduplicatedSamples_;
}

uint32_t HeadTelemetry::malformedSamples() const {
  HeadTelemetryLockGuard guard(lock_);
  return malformedSamples_;
}

uint32_t HeadTelemetry::rejectedSamples() const {
  HeadTelemetryLockGuard guard(lock_);
  return rejectedSamples_;
}

bool HeadTelemetry::hasSample(size_t streamIndex) const {
  HeadTelemetryLockGuard guard(lock_);
  return streamIndex < streamCount_ && streams_[streamIndex].registered &&
         streams_[streamIndex].sample.valid;
}

bool HeadTelemetry::samplePayloadEquals(size_t streamIndex,
                                        const uint8_t* payload,
                                        size_t payloadLength) const {
  if (payload == nullptr ||
      payloadLength > ardb_head::kMaxApplicationPayloadBytes) {
    return false;
  }

  HeadTelemetryLockGuard guard(lock_);
  if (streamIndex >= streamCount_ || !streams_[streamIndex].registered) {
    return false;
  }
  const SampleSlot& sample = streams_[streamIndex].sample;
  return sample.valid && sample.length == payloadLength &&
         (payloadLength == 0 ||
          memcmp(sample.payload, payload, payloadLength) == 0);
}

size_t HeadTelemetry::buildDefinitionBody(size_t definitionIndex,
                                          uint8_t* output,
                                          size_t outputCapacity) const {
  if (output == nullptr) {
    return 0;
  }

  HeadTelemetryLockGuard guard(lock_);
  if (definitionIndex >= streamCount_ ||
      !streams_[definitionIndex].registered) {
    return 0;
  }

  const StreamSlot& stream = streams_[definitionIndex];
  const size_t nameLength = strlen(stream.displayName);
  constexpr size_t kDefinitionPrefixBytes = 5;
  const size_t required = kDefinitionPrefixBytes + nameLength;
  if (outputCapacity < required) {
    return 0;
  }

  output[0] = stream.id;
  output[1] = stream.visualType;
  writeU16(&output[2], stream.expectedPayloadBytes);
  output[4] = static_cast<uint8_t>(nameLength);
  memcpy(&output[kDefinitionPrefixBytes], stream.displayName, nameLength);
  return required;
}

size_t HeadTelemetry::buildSampleRecord(size_t streamIndex, uint8_t* output,
                                        size_t outputCapacity) const {
  if (output == nullptr) {
    return 0;
  }

  HeadTelemetryLockGuard guard(lock_);
  if (streamIndex >= streamCount_ || !streams_[streamIndex].registered) {
    return 0;
  }
  const StreamSlot& stream = streams_[streamIndex];
  const SampleSlot& sample = stream.sample;
  constexpr size_t kSamplePrefixBytes = 5;
  const size_t required = kSamplePrefixBytes + sample.length;
  if (!sample.valid || outputCapacity < required) {
    return 0;
  }

  const uint32_t ageMs = millis() - sample.receivedAtMs;
  output[0] = stream.id;
  writeU16(&output[1],
           ageMs > 0xffffUL ? 0xffffU : static_cast<uint16_t>(ageMs));
  writeU16(&output[3], sample.length);
  if (sample.length != 0) {
    memcpy(&output[kSamplePrefixBytes], sample.payload, sample.length);
  }
  return required;
}

size_t HeadTelemetry::buildStatusBody(uint8_t* output,
                                      size_t outputCapacity) const {
  constexpr size_t kStatusBytes = 21;
  if (output == nullptr || outputCapacity < kStatusBytes) {
    return 0;
  }

  HeadTelemetryLockGuard guard(lock_);
  uint8_t validStreams = 0;
  for (size_t index = 0; index < streamCount_; ++index) {
    if (streams_[index].registered && streams_[index].sample.valid) {
      ++validStreams;
    }
  }

  output[0] = validStreams;
  writeU32(&output[1], acceptedSamples_);
  writeU32(&output[5], deduplicatedSamples_);
  writeU32(&output[9], malformedSamples_);
  writeU32(&output[13], rejectedSamples_);
  writeU32(&output[17], generation_);
  return kStatusBytes;
}

void HeadTelemetry::onMqttPublish(const MqttClient*, const Topic& topic,
                                  const char* payload,
                                  size_t payloadLength) {
  if (instance_ != nullptr) {
    instance_->ingest(topic, payload, payloadLength);
  }
}

void HeadTelemetry::ingest(const Topic& topic, const char* payload,
                           size_t payloadLength) {
  const char* topicText = topic.c_str();
  if (isMetadataTopic(topicText)) {
    ingestMetadata(payload, payloadLength);
    return;
  }
  ingestSample(topic, payload, payloadLength);
}

void HeadTelemetry::ingestMetadata(const char* payload, size_t payloadLength) {
  if (payload == nullptr || payloadLength < kMetadataHeaderBytes + kChecksumBytes) {
    HeadTelemetryLockGuard guard(lock_);
    ++malformedSamples_;
    return;
  }

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(payload);
  const uint16_t expectedCrc =
      static_cast<uint16_t>(bytes[payloadLength - 2] << 8) |
      static_cast<uint16_t>(bytes[payloadLength - 1]);
  if (crc16Ccitt(bytes, payloadLength - kChecksumBytes) != expectedCrc) {
    HeadTelemetryLockGuard guard(lock_);
    ++malformedSamples_;
    return;
  }

  if (memcmp(bytes, "ARDB", 4) != 0 || bytes[4] != kMetadataVersion) {
    HeadTelemetryLockGuard guard(lock_);
    ++rejectedSamples_;
    return;
  }

  const uint16_t expectedPayloadBytes =
      static_cast<uint16_t>(bytes[6] << 8) | static_cast<uint16_t>(bytes[7]);
  const size_t topicLength = bytes[8];
  const size_t nameLength = bytes[9];
  const size_t requiredLength =
      kMetadataHeaderBytes + topicLength + nameLength + kChecksumBytes;
  if (expectedPayloadBytes > ardb_head::kMaxApplicationPayloadBytes ||
      topicLength == 0 || topicLength > ardb_head::kMaxMqttTopicBytes ||
      nameLength == 0 || nameLength > ardb_head::kMaxDefinitionNameBytes ||
      payloadLength != requiredLength ||
      memchr(&bytes[kMetadataHeaderBytes], '\0', topicLength + nameLength) !=
          nullptr) {
    HeadTelemetryLockGuard guard(lock_);
    ++rejectedSamples_;
    return;
  }

  char descriptorTopic[ardb_head::kMaxMqttTopicBytes + 1];
  memcpy(descriptorTopic, &bytes[kMetadataHeaderBytes], topicLength);
  descriptorTopic[topicLength] = '\0';
  const char* descriptorName = reinterpret_cast<const char*>(
      &bytes[kMetadataHeaderBytes + topicLength]);

  HeadTelemetryLockGuard guard(lock_);
  int streamIndex = findStream(descriptorTopic);
  if (streamIndex < 0) {
    if (streamCount_ >= ardb_head::kMaxStreams) {
      ++rejectedSamples_;
      return;
    }
    streamIndex = static_cast<int>(streamCount_++);
    StreamSlot& stream = streams_[static_cast<size_t>(streamIndex)];
    memset(&stream, 0, sizeof(stream));
    stream.id = static_cast<uint8_t>(streamIndex + 1);
    stream.registered = true;
  }

  StreamSlot& stream = streams_[static_cast<size_t>(streamIndex)];
  const bool changed = stream.visualType != bytes[5] ||
                       stream.expectedPayloadBytes != expectedPayloadBytes ||
                       strncmp(stream.displayName, descriptorName, nameLength) != 0 ||
                       stream.displayName[nameLength] != '\0';
  if (!changed) {
    return;
  }

  memcpy(stream.topic, descriptorTopic, topicLength);
  stream.topic[topicLength] = '\0';
  memcpy(stream.displayName, descriptorName, nameLength);
  stream.displayName[nameLength] = '\0';
  stream.visualType = bytes[5];
  stream.expectedPayloadBytes = expectedPayloadBytes;
  stream.sample.valid = false;
  ++registryGeneration_;
}

void HeadTelemetry::ingestSample(const Topic& topic, const char* payload,
                                 size_t payloadLength) {
  HeadTelemetryLockGuard guard(lock_);
  const int streamIndex = findStream(topic.c_str());
  if (streamIndex < 0) {
    ++rejectedSamples_;
    return;
  }
  if (payload == nullptr || payloadLength < kChecksumBytes) {
    ++malformedSamples_;
    return;
  }

  const size_t applicationLength = payloadLength - kChecksumBytes;
  StreamSlot& stream = streams_[static_cast<size_t>(streamIndex)];
  if (applicationLength > ardb_head::kMaxApplicationPayloadBytes ||
      (stream.expectedPayloadBytes != 0 &&
       applicationLength != stream.expectedPayloadBytes)) {
    ++rejectedSamples_;
    return;
  }

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(payload);
  const uint16_t expectedCrc =
      static_cast<uint16_t>(bytes[payloadLength - 2] << 8) |
      static_cast<uint16_t>(bytes[payloadLength - 1]);
  if (crc16Ccitt(bytes, applicationLength) != expectedCrc) {
    ++malformedSamples_;
    return;
  }

  SampleSlot& sample = stream.sample;
  const bool isDuplicate =
      sample.valid && sample.length == applicationLength &&
      (applicationLength == 0 ||
       memcmp(sample.payload, bytes, applicationLength) == 0);

  ++acceptedSamples_;
  sample.receivedAtMs = millis();
  if (isDuplicate) {
    ++deduplicatedSamples_;
    return;
  }

  if (applicationLength != 0) {
    memcpy(sample.payload, bytes, applicationLength);
  }
  sample.length = static_cast<uint16_t>(applicationLength);
  sample.valid = true;
  ++generation_;
}

int HeadTelemetry::findStream(const char* topic) const {
  if (topic == nullptr) {
    return -1;
  }
  for (size_t index = 0; index < streamCount_; ++index) {
    if (streams_[index].registered && strcmp(topic, streams_[index].topic) == 0) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

bool HeadTelemetry::isMetadataTopic(const char* topic) {
  return topic != nullptr &&
         strncmp(topic, kMetadataTopicPrefix, kMetadataPrefixLength) == 0 &&
         topic[kMetadataPrefixLength] != '\0';
}

uint16_t HeadTelemetry::crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xffff;
  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void HeadTelemetry::writeU16(uint8_t* output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value >> 8);
  output[1] = static_cast<uint8_t>(value & 0xffU);
}

void HeadTelemetry::writeU32(uint8_t* output, uint32_t value) {
  output[0] = static_cast<uint8_t>(value >> 24);
  output[1] = static_cast<uint8_t>(value >> 16);
  output[2] = static_cast<uint8_t>(value >> 8);
  output[3] = static_cast<uint8_t>(value & 0xffU);
}
