#include "HeadStreamSession.h"

#include <string.h>

HeadStreamSession::HeadStreamSession(HeadTelemetry& telemetry)
    : telemetry_(telemetry),
      frameSequence_(0),
      lastDeltaAtMs_(0),
      lastStatusAtMs_(0),
      observedRegistryGeneration_(0),
      lastSentPayloads_{},
      lastSentPayloadLengths_{},
      lastSentValuesValid_{},
      pendingRecordOffsets_{},
      pendingSamples_{} {}

bool HeadStreamSession::begin(HeadByteWriter& writer, uint32_t nowMs) {
  memset(lastSentPayloads_, 0, sizeof(lastSentPayloads_));
  memset(lastSentPayloadLengths_, 0, sizeof(lastSentPayloadLengths_));
  memset(lastSentValuesValid_, 0, sizeof(lastSentValuesValid_));
  lastDeltaAtMs_ = nowMs;
  lastStatusAtMs_ = nowMs;

  if (!sendStatus(writer, nowMs) || !sendDefinitions(writer, nowMs) ||
      !sendBaseline(writer, nowMs)) {
    return false;
  }
  observedRegistryGeneration_ = telemetry_.registryGeneration();
  return true;
}

bool HeadStreamSession::update(HeadByteWriter& writer, uint32_t nowMs) {
  // Metadata can arrive after the Quest opened the live endpoint. Re-emit the
  // current definitions and a complete baseline before any data from the new
  // registry is delivered. Definitions are idempotent upserts by stream ID.
  const uint32_t registryGeneration = telemetry_.registryGeneration();
  if (registryGeneration != observedRegistryGeneration_) {
    if (!sendDefinitions(writer, nowMs) || !sendBaseline(writer, nowMs)) {
      return false;
    }
    observedRegistryGeneration_ = telemetry_.registryGeneration();
  }

  if (hasUnsentSamples() &&
      isDue(nowMs, lastDeltaAtMs_ + ardb_head::kDeltaMinIntervalMs)) {
    if (!sendDelta(writer, nowMs)) {
      return false;
    }
    lastDeltaAtMs_ = nowMs;
  }

  if (isDue(nowMs, lastStatusAtMs_ + ardb_head::kStatusPeriodMs)) {
    if (!sendStatus(writer, nowMs)) {
      return false;
    }
    lastStatusAtMs_ = nowMs;
  }
  return true;
}

bool HeadStreamSession::needsService(uint32_t nowMs) const {
  return telemetry_.registryGeneration() != observedRegistryGeneration_ ||
         (hasUnsentSamples() &&
          isDue(nowMs, lastDeltaAtMs_ + ardb_head::kDeltaMinIntervalMs)) ||
         isDue(nowMs, lastStatusAtMs_ + ardb_head::kStatusPeriodMs);
}

bool HeadStreamSession::sendDefinitions(HeadByteWriter& writer,
                                        uint32_t nowMs) {
  uint8_t body[ardb_head::kMaxFrameBodyBytes];
  for (size_t index = 0; index < telemetry_.streamCount(); ++index) {
    const size_t bodyLength =
        telemetry_.buildDefinitionBody(index, body, sizeof(body));
    if (bodyLength == 0 ||
        !ardb_head::writeChunkedFrame(
            writer, ardb_head::FrameType::StreamDefinition, ++frameSequence_,
            nowMs, body, bodyLength)) {
      return false;
    }
  }
  return true;
}

bool HeadStreamSession::sendBaseline(HeadByteWriter& writer, uint32_t nowMs) {
  // A baseline supersedes the session cache. Clearing it first also lets one
  // bounded frame batch cover an arbitrary configured stream capacity.
  memset(lastSentValuesValid_, 0, sizeof(lastSentValuesValid_));
  return sendSampleBatch(writer, ardb_head::FrameType::Baseline, nowMs);
}

bool HeadStreamSession::sendDelta(HeadByteWriter& writer, uint32_t nowMs) {
  return sendSampleBatch(writer, ardb_head::FrameType::Delta, nowMs);
}

bool HeadStreamSession::sendSampleBatch(HeadByteWriter& writer,
                                        ardb_head::FrameType type,
                                        uint32_t nowMs) {
  bool wroteFrame = false;
  while (true) {
    uint8_t body[ardb_head::kMaxFrameBodyBytes];
    body[0] = 0;
    size_t bodyLength = 1;
    uint8_t sampleCount = 0;
    memset(pendingSamples_, 0, sizeof(pendingSamples_));

    const size_t streamCount = telemetry_.streamCount();
    for (size_t index = 0; index < streamCount; ++index) {
      if (!telemetry_.hasSample(index) ||
          (lastSentValuesValid_[index] &&
           telemetry_.samplePayloadEquals(index, lastSentPayloads_[index],
                                          lastSentPayloadLengths_[index]))) {
        continue;
      }

      const size_t remaining = sizeof(body) - bodyLength;
      if (remaining < 4) {
        break;
      }
      pendingRecordOffsets_[index] = static_cast<uint16_t>(bodyLength);
      const size_t recordLength = telemetry_.buildSampleRecord(
          index, &body[bodyLength], remaining);
      if (recordLength == 0) {
        if (sampleCount == 0) {
          return false;
        }
        break;
      }
      bodyLength += recordLength;
      ++sampleCount;
      pendingSamples_[index] = true;
    }

    body[0] = sampleCount;
    if (sampleCount == 0 && wroteFrame) {
      return true;
    }
    if (!ardb_head::writeChunkedFrame(writer, type, ++frameSequence_, nowMs,
                                      body, bodyLength)) {
      return false;
    }
    wroteFrame = true;

    for (size_t index = 0; index < streamCount; ++index) {
      if (!pendingSamples_[index]) {
        continue;
      }

      constexpr size_t kSamplePayloadLengthOffset = 3;
      constexpr size_t kSamplePayloadOffset = 4;
      const size_t offset = pendingRecordOffsets_[index];
      const uint8_t payloadLength = body[offset + kSamplePayloadLengthOffset];
      if (payloadLength != 0) {
        memcpy(lastSentPayloads_[index], &body[offset + kSamplePayloadOffset],
               payloadLength);
      }
      lastSentPayloadLengths_[index] = payloadLength;
      lastSentValuesValid_[index] = true;
    }

    if (sampleCount == 0) {
      return true;
    }
  }
}

bool HeadStreamSession::sendStatus(HeadByteWriter& writer, uint32_t nowMs) {
  uint8_t body[ardb_head::kMaxFrameBodyBytes];
  const size_t bodyLength = telemetry_.buildStatusBody(body, sizeof(body));
  return bodyLength != 0 &&
         ardb_head::writeChunkedFrame(writer, ardb_head::FrameType::Status,
                                      ++frameSequence_, nowMs, body,
                                      bodyLength);
}

bool HeadStreamSession::hasUnsentSamples() const {
  for (size_t index = 0; index < telemetry_.streamCount(); ++index) {
    if (telemetry_.hasSample(index) &&
        (!lastSentValuesValid_[index] ||
         !telemetry_.samplePayloadEquals(index, lastSentPayloads_[index],
                                         lastSentPayloadLengths_[index]))) {
      return true;
    }
  }
  return false;
}

bool HeadStreamSession::isDue(uint32_t nowMs, uint32_t targetMs) {
  return static_cast<int32_t>(nowMs - targetMs) >= 0;
}
