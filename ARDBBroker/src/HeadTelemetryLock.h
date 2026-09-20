#pragma once

// Pico services MQTT and HTTPS cooperatively in one loop. ESP32's native HTTP
// server runs its callbacks in a separate task, so the same telemetry cache
// needs a very small critical section there. Keeping this distinction here
// prevents the protocol and cache code from depending on either SDK directly.
#if defined(ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#endif

class HeadTelemetryLock final {
 public:
  HeadTelemetryLock()
#if defined(ESP32)
      : lock_(portMUX_INITIALIZER_UNLOCKED)
#endif
  {}

  void lock() const {
#if defined(ESP32)
    portENTER_CRITICAL(&lock_);
#endif
  }

  void unlock() const {
#if defined(ESP32)
    portEXIT_CRITICAL(&lock_);
#endif
  }

 private:
#if defined(ESP32)
  mutable portMUX_TYPE lock_;
#endif
};

class HeadTelemetryLockGuard final {
 public:
  explicit HeadTelemetryLockGuard(const HeadTelemetryLock& lock) : lock_(lock) {
    lock_.lock();
  }

  ~HeadTelemetryLockGuard() { lock_.unlock(); }

  HeadTelemetryLockGuard(const HeadTelemetryLockGuard&) = delete;
  HeadTelemetryLockGuard& operator=(const HeadTelemetryLockGuard&) = delete;

 private:
  const HeadTelemetryLock& lock_;
};
