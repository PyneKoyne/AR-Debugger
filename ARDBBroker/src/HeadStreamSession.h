#pragma once

#include <Arduino.h>

#include "HeadByteWriter.h"
#include "HeadConfig.h"
#include "HeadProtocol.h"
#include "HeadTelemetry.h"

// Owns the AHSP/4 session sequence and delta cache. It is portable across
// Arduino targets: platform code only decides how bytes reach an HTTPS peer.
class HeadStreamSession {
 public:
  explicit HeadStreamSession(HeadTelemetry& telemetry);

  // Sends the required initial status, definitions, and baseline. A session
  // can begin only after the HTTP transport has sent its response headers.
  bool begin(HeadByteWriter& writer, uint32_t nowMs);

  // Sends any due delta and heartbeat. Returns false after a failed write.
  bool update(HeadByteWriter& writer, uint32_t nowMs);

  // Lets a platform avoid scheduling unnecessary write work.
  bool needsService(uint32_t nowMs) const;

 private:
  bool sendDefinitions(HeadByteWriter& writer, uint32_t nowMs);
  bool sendBaseline(HeadByteWriter& writer, uint32_t nowMs);
  bool sendDelta(HeadByteWriter& writer, uint32_t nowMs);
  bool sendSampleBatch(HeadByteWriter& writer, ardb_head::FrameType type,
                       uint32_t nowMs);
  bool sendStatus(HeadByteWriter& writer, uint32_t nowMs);
  bool hasUnsentSamples() const;
  static bool isDue(uint32_t nowMs, uint32_t targetMs);

  HeadTelemetry& telemetry_;
  uint32_t frameSequence_;
  uint32_t lastDeltaAtMs_;
  uint32_t lastStatusAtMs_;
  uint32_t observedRegistryGeneration_;
  uint8_t lastSentPayloads_[ardb_head::kMaxStreams]
                           [ardb_head::kMaxApplicationPayloadBytes];
  uint16_t lastSentPayloadLengths_[ardb_head::kMaxStreams];
  bool lastSentValuesValid_[ardb_head::kMaxStreams];
  uint16_t pendingRecordOffsets_[ardb_head::kMaxStreams];
  bool pendingSamples_[ardb_head::kMaxStreams];
};
