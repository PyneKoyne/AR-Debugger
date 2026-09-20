#pragma once

#include <stddef.h>
#include <stdint.h>

// The AHSP encoder deliberately knows nothing about TLS, HTTP servers, Wi-Fi,
// or Arduino Client subclasses. A platform adapter supplies this one operation.
class HeadByteWriter {
 public:
  virtual ~HeadByteWriter() = default;

  // Returns true only when every requested byte was accepted by the active
  // connection. Implementations must not retain the supplied buffer.
  virtual bool writeAll(const uint8_t* bytes, size_t length) = 0;
};
