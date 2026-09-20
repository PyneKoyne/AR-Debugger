#pragma once

#include <Arduino.h>

#include "HeadConfig.h"
#include "HeadSecureServer.h"
#include "HeadStreamSession.h"
#include "HeadTelemetry.h"

// Arduino-Pico TLS transport adapter. The AHSP/3 encoder and session policy
// remain in the target-independent HeadStreamSession class.
class HeadHttpsStream final {
 public:
  HeadHttpsStream(HeadTelemetry& telemetry, const char* certificatePem,
                  const char* privateKeyPem);

  void begin();
  void update();

 private:
  enum class ConnectionState : uint8_t {
    Idle,
    ReadingRequest,
    Streaming,
  };

  void acceptConnection();
  void readRequest();
  void handleCompleteRequest();
  void serviceLiveStream();
  void startLiveStream();

  bool sendHealth();
  bool sendError(const char* status, const char* body);
  bool writeHttpHeader(const char* status, const char* contentType,
                       size_t contentLength, bool chunked);
  bool writeAll(const uint8_t* data, size_t length);
  void closeConnection();

  HeadTelemetry& telemetry_;
  HeadSecureServer server_;
  BearSSL::X509List certificate_;
  BearSSL::PrivateKey privateKey_;
  BearSSL::ServerSession sessionStorage_[1];
  BearSSL::ServerSessions sessionCache_;
  BearSSL::WiFiClientSecure client_;
  HeadStreamSession streamSession_;

  ConnectionState state_;
  char request_[ardb_head::kMaxHttpRequestBytes];
  size_t requestLength_;
  uint32_t requestDeadlineMs_;
};
