#pragma once

#include <Arduino.h>

#include "HeadConfig.h"
#include "HeadTelemetryLock.h"
#include "TinyMqtt.h"

// A bounded, dynamically populated registry. Stream descriptors arrive on
// ardb/meta/<publisher>/<descriptor> and consume one preallocated slot. No
// heap allocation occurs while registering or accepting telemetry.
class HeadTelemetry {
 public:
  explicit HeadTelemetry(MqttBroker& broker);

  // Subscribes to all broker traffic so newly advertised stream topics do not
  // require a late, per-topic subscription.
  void begin();

  size_t streamCount() const;
  uint32_t generation() const;
  uint32_t registryGeneration() const;
  uint32_t acceptedSamples() const;
  uint32_t deduplicatedSamples() const;
  uint32_t malformedSamples() const;
  uint32_t rejectedSamples() const;

  bool hasSample(size_t streamIndex) const;
  bool samplePayloadEquals(size_t streamIndex, const uint8_t* payload,
                           size_t payloadLength) const;

  // Bodies are binary and do not include a live-stream frame header or CRC.
  // The return value is the number of bytes written, or zero on insufficient
  // output capacity. Definition index is zero-based.
  size_t buildDefinitionBody(size_t definitionIndex, uint8_t* output,
                             size_t outputCapacity) const;
  // Encodes one entry used by both BASELINE and DELTA frame bodies:
  // stream ID:u8 | sample age milliseconds:u16 BE | payload length:u16 BE |
  // application bytes.
  size_t buildSampleRecord(size_t streamIndex, uint8_t* output,
                           size_t outputCapacity) const;
  size_t buildStatusBody(uint8_t* output, size_t outputCapacity) const;

 private:
  struct SampleSlot {
    uint8_t payload[ardb_head::kMaxApplicationPayloadBytes];
    uint16_t length;
    uint32_t receivedAtMs;
    bool valid;
  };

  struct StreamSlot {
    char topic[ardb_head::kMaxMqttTopicBytes + 1];
    char displayName[ardb_head::kMaxDefinitionNameBytes + 1];
    uint8_t id;
    uint8_t visualType;
    uint16_t expectedPayloadBytes;
    SampleSlot sample;
    bool registered;
  };

  static void onMqttPublish(const MqttClient* source, const Topic& topic,
                            const char* payload, size_t payloadLength);
  void ingest(const Topic& topic, const char* payload, size_t payloadLength);
  void ingestMetadata(const char* payload, size_t payloadLength);
  void ingestSample(const Topic& topic, const char* payload,
                    size_t payloadLength);
  int findStream(const char* topic) const;
  static bool isMetadataTopic(const char* topic);

  static uint16_t crc16Ccitt(const uint8_t* data, size_t length);
  static void writeU16(uint8_t* output, uint16_t value);
  static void writeU32(uint8_t* output, uint32_t value);

  MqttBroker& broker_;
  MqttClient subscriber_;
  StreamSlot streams_[ardb_head::kMaxStreams];
  size_t streamCount_;
  uint32_t generation_;
  uint32_t registryGeneration_;
  uint32_t acceptedSamples_;
  uint32_t deduplicatedSamples_;
  uint32_t malformedSamples_;
  uint32_t rejectedSamples_;
  mutable HeadTelemetryLock lock_;

  static HeadTelemetry* instance_;
};
