#pragma once

#if defined(ARDUINO_ARCH_RP2040)

#include <WiFiServerSecure.h>

// Arduino-Pico 6.1.0's BearSSL server accepts a pending ClientContext without
// releasing lwIP's delayed listen-backlog slot. This wrapper performs that
// missing acknowledgement before delegating to the core's TLS accept path.
// Keep this workaround pinned to the tested Arduino-Pico core version; a core
// upgrade must be reviewed to avoid acknowledging the same slot twice.
class HeadSecureServer final : public BearSSL::WiFiServerSecure {
 public:
  explicit HeadSecureServer(uint16_t port);

  BearSSL::WiFiClientSecure accept();
};

#endif  // defined(ARDUINO_ARCH_RP2040)
