#pragma once

// The public head-node interface is intentionally stable. Each supported
// target selects an HTTPS transport adapter, while AHSP/3 session behavior is
// shared by HeadStreamSession.
#if defined(ARDUINO_ARCH_RP2040)
#include "HeadHttpsStreamPico.h"
#elif defined(ESP32)
#include "HeadHttpsStreamEsp32.h"
#else
#error "ARDBBroker supports Arduino-Pico RP2040/RP2350 Wi-Fi boards and ESP32 Arduino targets"
#endif
