#pragma once

#include <Arduino.h>

// The head discovers streams from retained ARDB metadata. This capacity is a
// compile-time resource budget selected by the firmware integrator. It is not
// a network-controlled allocation limit: every slot and payload buffer is
// statically reserved when the sketch is compiled.
namespace ardb_head {

constexpr uint16_t kApiPort = 443;
constexpr uint8_t kLiveProtocolVersion = 3;
constexpr char kLivePath[] = "/api/v2/live";
constexpr char kHealthPath[] = "/api/v2/health";

// A changed stream is emitted no more than once during this period. When a
// publisher updates faster, only its most recent valid value is retained.
constexpr uint32_t kDeltaMinIntervalMs = 100;
constexpr uint32_t kStatusPeriodMs = 5000;
constexpr uint32_t kRequestTimeoutMs = 3000;
constexpr uint32_t kTlsCloseFlushTimeoutMs = 20;

constexpr size_t kMaxApplicationPayloadBytes = 64;
constexpr size_t kMaxDefinitionNameBytes = 48;
constexpr size_t kMaxMqttTopicBytes = 64;
constexpr size_t kMaxFrameBodyBytes = 192;
constexpr size_t kMaxHttpRequestBytes = 384;

// This is a development-only default for a read-only API with no credentials.
// Set it to the exact HTTPS origin of the Quest application before distributing
// the debugger beyond a controlled lab.
constexpr const char kCorsAllowOrigin[] = "*";

// The event logger is useful when bringing up a publisher, but printing every
// MQTT message is intentionally disabled in the streaming performance profile.
// This is a preprocessor switch because the logger object must not be created
// at all when disabled.
#ifndef ARDB_HEAD_ENABLE_MQTT_EVENT_LOGGER
#define ARDB_HEAD_ENABLE_MQTT_EVENT_LOGGER 0
#endif

#ifndef ARDB_HEAD_MAX_STREAMS
#define ARDB_HEAD_MAX_STREAMS 8
#endif

constexpr size_t kMaxStreams = ARDB_HEAD_MAX_STREAMS;
static_assert(kMaxStreams > 0, "ARDB_HEAD_MAX_STREAMS must be positive");
static_assert(kMaxStreams <= 255,
              "ARDB_HEAD_MAX_STREAMS must fit in an 8-bit stream ID");

}  // namespace ardb_head
