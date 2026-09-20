#pragma once

#include <Arduino.h>

namespace ardb_head {

// Heap is the portable runtime-memory metric exposed by the health endpoint.
// It excludes static allocations and task stacks, which the supported cores do
// not expose through one comparable API.
struct HeapMemoryStats {
  uint32_t totalBytes;
  uint32_t usedBytes;
  uint32_t freeBytes;
};

inline HeapMemoryStats readHeapMemoryStats() {
#if defined(ESP32)
  const uint32_t totalBytes = ESP.getHeapSize();
  const uint32_t freeBytes = ESP.getFreeHeap();
  return {totalBytes, totalBytes >= freeBytes ? totalBytes - freeBytes : 0,
          freeBytes};
#elif defined(ARDUINO_ARCH_RP2040)
  const int totalBytes = rp2040.getTotalHeap();
  const int usedBytes = rp2040.getUsedHeap();
  const int freeBytes = rp2040.getFreeHeap();
  return {totalBytes > 0 ? static_cast<uint32_t>(totalBytes) : 0,
          usedBytes > 0 ? static_cast<uint32_t>(usedBytes) : 0,
          freeBytes > 0 ? static_cast<uint32_t>(freeBytes) : 0};
#else
#error "ARDBBroker health-memory metrics require ESP32 or Arduino-Pico"
#endif
}

}  // namespace ardb_head
