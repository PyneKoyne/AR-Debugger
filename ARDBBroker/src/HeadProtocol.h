#pragma once

#include <stddef.h>
#include <stdint.h>

#include "HeadByteWriter.h"
#include "HeadConfig.h"

namespace ardb_head {

enum class FrameType : uint8_t {
  Status = 1,
  StreamDefinition = 2,
  // Sent once after definitions. It establishes the complete cached state
  // available when this HTTP connection began.
  Baseline = 3,
  // Sent afterward only for streams whose application payload changed.
  Delta = 4,
};

// Writes one complete application frame as a single HTTP chunk. The frame is
// independently delimited and checksummed because Fetch stream reads are not
// aligned to HTTP chunk boundaries.
bool writeChunkedFrame(HeadByteWriter& writer, FrameType type, uint32_t sequence,
                       uint32_t timestampMs, const uint8_t* body,
                       size_t bodyLength);

}  // namespace ardb_head
