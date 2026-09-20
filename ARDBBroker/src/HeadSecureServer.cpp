#include "HeadSecureServer.h"

#if defined(ARDUINO_ARCH_RP2040)

#include <RP2040Version.h>
#include <include/ClientContext.h>
#include <lwip/tcp.h>

static_assert(ARDUINO_PICO_MAJOR == 6 && ARDUINO_PICO_MINOR == 1 &&
                  ARDUINO_PICO_REVISION == 0,
              "Review HeadSecureServer before changing the Arduino-Pico core");

HeadSecureServer::HeadSecureServer(uint16_t port)
    : BearSSL::WiFiServerSecure(port) {}

BearSSL::WiFiClientSecure HeadSecureServer::accept() {
  // WiFiServer::_accept() calls tcp_backlog_delayed() for every pending TCP
  // peer. The non-secure WiFiServer::accept() balances it with this call, but
  // Arduino-Pico 6.1.0's WiFiServerSecure::accept() does not. Acknowledge the
  // pending slot before the base class removes it from _unclaimed. Doing this
  // before TLS setup also remains safe if the handshake itself fails and
  // destroys the ClientContext.
  if (_unclaimed != nullptr) {
    tcp_pcb* const pendingPcb = _unclaimed->getPCB();
    if (pendingPcb != nullptr) {
      tcp_backlog_accepted(pendingPcb);
    }
  }

  return BearSSL::WiFiServerSecure::accept();
}

#endif  // defined(ARDUINO_ARCH_RP2040)
