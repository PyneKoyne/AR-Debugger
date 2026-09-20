#include "HeadProtocol.h"

#include <stdio.h>
#include <string.h>

namespace ardb_head {
namespace {

constexpr uint16_t kFrameMagic = 0xa7db;
constexpr size_t kFrameHeaderBytes = 14;
constexpr size_t kFrameCrcBytes = 2;
constexpr size_t kMaxFrameBytes =
    kFrameHeaderBytes + kMaxFrameBodyBytes + kFrameCrcBytes;

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
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

void writeU16(uint8_t* output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value >> 8);
  output[1] = static_cast<uint8_t>(value & 0xffU);
}

void writeU32(uint8_t* output, uint32_t value) {
  output[0] = static_cast<uint8_t>(value >> 24);
  output[1] = static_cast<uint8_t>(value >> 16);
  output[2] = static_cast<uint8_t>(value >> 8);
  output[3] = static_cast<uint8_t>(value & 0xffU);
}

}  // namespace

bool writeChunkedFrame(HeadByteWriter& writer, FrameType type,
                       uint32_t sequence,
                       uint32_t timestampMs, const uint8_t* body,
                       size_t bodyLength) {
  if ((body == nullptr && bodyLength != 0) || bodyLength > kMaxFrameBodyBytes) {
    return false;
  }

  uint8_t frame[kMaxFrameBytes];
  writeU16(&frame[0], kFrameMagic);
  frame[2] = kLiveProtocolVersion;
  frame[3] = static_cast<uint8_t>(type);
  writeU32(&frame[4], sequence);
  writeU32(&frame[8], timestampMs);
  writeU16(&frame[12], static_cast<uint16_t>(bodyLength));
  if (bodyLength != 0) {
    memcpy(&frame[kFrameHeaderBytes], body, bodyLength);
  }

  const size_t withoutCrc = kFrameHeaderBytes + bodyLength;
  writeU16(&frame[withoutCrc], crc16Ccitt(frame, withoutCrc));
  const size_t frameLength = withoutCrc + kFrameCrcBytes;

  char chunkHeader[12];
  const int chunkHeaderLength =
      snprintf(chunkHeader, sizeof(chunkHeader), "%X\r\n",
               static_cast<unsigned int>(frameLength));
  if (chunkHeaderLength <= 0 ||
      static_cast<size_t>(chunkHeaderLength) >= sizeof(chunkHeader)) {
    return false;
  }

  const uint8_t chunkTerminator[] = {'\r', '\n'};
  return writer.writeAll(reinterpret_cast<const uint8_t*>(chunkHeader),
                         static_cast<size_t>(chunkHeaderLength)) &&
         writer.writeAll(frame, frameLength) &&
         writer.writeAll(chunkTerminator, sizeof(chunkTerminator));
}

}  // namespace ardb_head
