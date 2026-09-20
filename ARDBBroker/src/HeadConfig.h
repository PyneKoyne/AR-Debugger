#pragma once

#include <Arduino.h>

// The head discovers streams from retained ARDB metadata. This capacity is a
// compile-time resource budget selected by the firmware integrator. It is not
// a network-controlled allocation limit: every slot and payload buffer is
// statically reserved when the sketch is compiled.
namespace ardb_head {

constexpr uint16_t kApiPort = 443;
// Version 4 widens BASELINE/DELTA application-payload lengths from u8 to u16.
// Quest clients must reject prior frame versions rather than misreading them.
constexpr uint8_t kLiveProtocolVersion = 4;
constexpr char kLivePath[] = "/api/v2/live";
constexpr char kHealthPath[] = "/api/v2/health";

// A changed stream is emitted no more than once during this period. When a
// publisher updates faster, only its most recent valid value is retained.
constexpr uint32_t kDeltaMinIntervalMs = 100;
constexpr uint32_t kStatusPeriodMs = 5000;
constexpr uint32_t kRequestTimeoutMs = 3000;
constexpr uint32_t kTlsCloseFlushTimeoutMs = 20;

// Ordinary telemetry is capped at 64 bytes. JPEG is the sole exception: the
// image path may carry one 16-kilobit (2,048-byte) compressed image.
constexpr size_t kMaxApplicationPayloadBytes = 64;
constexpr size_t kMaxJpegApplicationPayloadBytes = 2048;
constexpr uint8_t kJpegVisualType = 7;
constexpr size_t kMaxDefinitionNameBytes = 48;
constexpr size_t kMaxMqttTopicBytes = 64;
// A sample frame has a one-byte count followed by a five-byte record header.
// Keep enough body space for one maximum-sized JPEG application payload.
constexpr size_t kMaxFrameBodyBytes = kMaxJpegApplicationPayloadBytes + 6;
constexpr size_t kMaxHttpRequestBytes = 384;

static_assert(kMaxFrameBodyBytes >= kMaxJpegApplicationPayloadBytes + 6,
              "frame body must hold a count and one maximum-sized JPEG sample");

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

// In access-point mode, answer the operating system's plain-HTTP network
// validation probe locally. This keeps local-only clients associated without
// pretending to proxy Internet traffic. Disable only when another DNS server
// owns port 53 on the AP network.
#ifndef ARDB_HEAD_ENABLE_LOCAL_NETWORK_VALIDATION
#define ARDB_HEAD_ENABLE_LOCAL_NETWORK_VALIDATION 1
#endif

#ifndef ARDB_HEAD_MAX_STREAMS
#define ARDB_HEAD_MAX_STREAMS 8
#endif

constexpr size_t kMaxStreams = ARDB_HEAD_MAX_STREAMS;
static_assert(kMaxStreams > 0, "ARDB_HEAD_MAX_STREAMS must be positive");
static_assert(kMaxStreams <= 255,
              "ARDB_HEAD_MAX_STREAMS must fit in an 8-bit stream ID");

}  // namespace ardb_head
