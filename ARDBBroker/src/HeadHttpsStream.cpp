#if defined(ARDUINO_ARCH_RP2040)

#include "HeadHttpsStreamPico.h"

#include <stdio.h>
#include <string.h>

#include "HeadByteWriter.h"

namespace {

constexpr uint8_t kRequestReadBudgetBytes = 96;

bool isDue(uint32_t nowMs, uint32_t targetMs) {
  return static_cast<int32_t>(nowMs - targetMs) >= 0;
}

bool requestTargets(const char* request, size_t requestLength,
                    const char* expectedPath) {
  const size_t expectedPathLength = strlen(expectedPath);
  const size_t minimumLength = 4 + expectedPathLength + 1;
  if (requestLength < minimumLength || strncmp(request, "GET ", 4) != 0) {
    return false;
  }

  const char* path = request + 4;
  return memcmp(path, expectedPath, expectedPathLength) == 0 &&
         path[expectedPathLength] == ' ';
}

class PicoClientWriter final : public HeadByteWriter {
 public:
  explicit PicoClientWriter(BearSSL::WiFiClientSecure& client)
      : client_(client) {}

  bool writeAll(const uint8_t* bytes, size_t length) override {
    return client_.write(bytes, length) == length;
  }

 private:
  BearSSL::WiFiClientSecure& client_;
};

}  // namespace

HeadHttpsStream::HeadHttpsStream(HeadTelemetry& telemetry,
                                 const char* certificatePem,
                                 const char* privateKeyPem)
    : telemetry_(telemetry),
      server_(ardb_head::kApiPort),
      certificate_(certificatePem),
      privateKey_(privateKeyPem),
      sessionStorage_{},
      sessionCache_(sessionStorage_, 1),
      client_(),
      streamSession_(telemetry),
      state_(ConnectionState::Idle),
      request_{},
      requestLength_(0),
      requestDeadlineMs_(0) {}

void HeadHttpsStream::begin() {
  // One ECDSA certificate and one cached session minimise reconnect cost while
  // preserving a single active API consumer.
  server_.setECCert(&certificate_, BR_KEYTYPE_KEYX | BR_KEYTYPE_SIGN,
                     &privateKey_);
  server_.setCache(&sessionCache_);
  server_.setSSLVersion(BR_TLS12, BR_TLS12);
  server_.begin();
}

void HeadHttpsStream::update() {
  switch (state_) {
    case ConnectionState::Idle:
      acceptConnection();
      break;
    case ConnectionState::ReadingRequest:
      readRequest();
      break;
    case ConnectionState::Streaming:
      serviceLiveStream();
      break;
  }
}

void HeadHttpsStream::acceptConnection() {
  BearSSL::WiFiClientSecure incoming = server_.accept();
  if (!incoming) {
    return;
  }

  client_ = incoming;
  state_ = ConnectionState::ReadingRequest;
  requestLength_ = 0;
  request_[0] = '\0';
  requestDeadlineMs_ = millis() + ardb_head::kRequestTimeoutMs;
}

void HeadHttpsStream::readRequest() {
  if (!client_.connected()) {
    closeConnection();
    return;
  }

  uint8_t consumed = 0;
  while (client_.available() > 0 && consumed < kRequestReadBudgetBytes) {
    const int next = client_.read();
    if (next < 0) {
      closeConnection();
      return;
    }
    ++consumed;

    if (requestLength_ + 1 >= sizeof(request_)) {
      sendError("431 Request Header Fields Too Large", "request too large\n");
      closeConnection();
      return;
    }

    request_[requestLength_++] = static_cast<char>(next);
    request_[requestLength_] = '\0';
    if (requestLength_ >= 4 &&
        memcmp(&request_[requestLength_ - 4], "\r\n\r\n", 4) == 0) {
      handleCompleteRequest();
      return;
    }
  }

  if (isDue(millis(), requestDeadlineMs_)) {
    sendError("408 Request Timeout", "request timeout\n");
    closeConnection();
  }
}

void HeadHttpsStream::handleCompleteRequest() {
  if (requestTargets(request_, requestLength_, ardb_head::kLivePath)) {
    startLiveStream();
    return;
  }

  if (requestTargets(request_, requestLength_, ardb_head::kHealthPath)) {
    sendHealth();
    closeConnection();
    return;
  }

  sendError("404 Not Found", "not found\n");
  closeConnection();
}

void HeadHttpsStream::startLiveStream() {
  if (!writeHttpHeader("200 OK", "application/octet-stream", 0, true)) {
    closeConnection();
    return;
  }

  state_ = ConnectionState::Streaming;
  PicoClientWriter writer(client_);
  if (!streamSession_.begin(writer, millis())) {
    closeConnection();
  }
}

void HeadHttpsStream::serviceLiveStream() {
  if (!client_.connected()) {
    closeConnection();
    return;
  }

  // The live endpoint is intentionally one-way. A request body or a second
  // pipelined request is rejected instead of accumulating inbound TLS data.
  if (client_.available() > 0) {
    closeConnection();
    return;
  }

  PicoClientWriter writer(client_);
  if (!streamSession_.update(writer, millis())) {
    closeConnection();
  }
}

bool HeadHttpsStream::sendHealth() {
  char body[192];
  const int bodyLength =
      snprintf(body, sizeof(body),
               "{\"protocol\":%u,\"streams\":%u,\"accepted\":%lu,"
               "\"deduplicated\":%lu,\"malformed\":%lu,"
               "\"rejected\":%lu}\n",
               static_cast<unsigned int>(ardb_head::kLiveProtocolVersion),
               static_cast<unsigned int>(telemetry_.streamCount()),
               static_cast<unsigned long>(telemetry_.acceptedSamples()),
               static_cast<unsigned long>(telemetry_.deduplicatedSamples()),
               static_cast<unsigned long>(telemetry_.malformedSamples()),
               static_cast<unsigned long>(telemetry_.rejectedSamples()));
  if (bodyLength <= 0 || static_cast<size_t>(bodyLength) >= sizeof(body) ||
      !writeHttpHeader("200 OK", "application/json",
                       static_cast<size_t>(bodyLength), false)) {
    return false;
  }
  return writeAll(reinterpret_cast<const uint8_t*>(body),
                  static_cast<size_t>(bodyLength));
}

bool HeadHttpsStream::sendError(const char* status, const char* body) {
  const size_t bodyLength = strlen(body);
  return writeHttpHeader(status, "text/plain; charset=utf-8", bodyLength,
                         false) &&
         writeAll(reinterpret_cast<const uint8_t*>(body), bodyLength);
}

bool HeadHttpsStream::writeHttpHeader(const char* status, const char* contentType,
                                      size_t contentLength, bool chunked) {
  char header[320];
  const int headerLength =
      chunked
          ? snprintf(header, sizeof(header),
                     "HTTP/1.1 %s\r\n"
                     "Content-Type: %s\r\n"
                     "Transfer-Encoding: chunked\r\n"
                     "Cache-Control: no-store\r\n"
                     "Access-Control-Allow-Origin: %s\r\n"
                     "X-Content-Type-Options: nosniff\r\n"
                     "Connection: keep-alive\r\n\r\n",
                     status, contentType, ardb_head::kCorsAllowOrigin)
          : snprintf(header, sizeof(header),
                     "HTTP/1.1 %s\r\n"
                     "Content-Type: %s\r\n"
                     "Content-Length: %lu\r\n"
                     "Cache-Control: no-store\r\n"
                     "Access-Control-Allow-Origin: %s\r\n"
                     "X-Content-Type-Options: nosniff\r\n"
                     "Connection: close\r\n\r\n",
                     status, contentType,
                     static_cast<unsigned long>(contentLength),
                     ardb_head::kCorsAllowOrigin);
  if (headerLength <= 0 || static_cast<size_t>(headerLength) >= sizeof(header)) {
    return false;
  }
  return writeAll(reinterpret_cast<const uint8_t*>(header),
                  static_cast<size_t>(headerLength));
}

bool HeadHttpsStream::writeAll(const uint8_t* data, size_t length) {
  return client_.write(data, length) == length;
}

void HeadHttpsStream::closeConnection() {
  // A finite flush budget prevents an unresponsive peer from monopolising the
  // cooperative broker loop during teardown. Resetting the wrapper immediately
  // releases its ClientContext and TLS buffers before the next accept.
  client_.stop(ardb_head::kTlsCloseFlushTimeoutMs);
  client_ = BearSSL::WiFiClientSecure();
  state_ = ConnectionState::Idle;
  requestLength_ = 0;
  request_[0] = '\0';
}

#endif  // defined(ARDUINO_ARCH_RP2040)
