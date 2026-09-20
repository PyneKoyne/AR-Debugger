#if defined(ESP32)

#include "HeadHttpsStreamEsp32.h"

#include <esp_https_server.h>

#include <stdio.h>
#include <string.h>

#include "HeadByteWriter.h"

namespace {

// ESP-IDF executes queued HTTP work in its own server task. Servicing no more
// often than the shared delta interval avoids sharing HeadStreamSession state
// with Arduino's loop task and prevents an idle live client from creating a
// high-rate queue of no-op server callbacks.
constexpr uint32_t kServicePollIntervalMs = ardb_head::kDeltaMinIntervalMs;

bool isDue(uint32_t nowMs, uint32_t targetMs) {
  return static_cast<int32_t>(nowMs - targetMs) >= 0;
}

class HttpdRequestWriter final : public HeadByteWriter {
 public:
  explicit HttpdRequestWriter(httpd_req_t& request) : request_(request) {}

  bool writeAll(const uint8_t* bytes, size_t length) override {
    return httpd_send(&request_, reinterpret_cast<const char*>(bytes), length) ==
           static_cast<int>(length);
  }

 private:
  httpd_req_t& request_;
};

class HttpdSocketWriter final : public HeadByteWriter {
 public:
  HttpdSocketWriter(httpd_handle_t server, int socket)
      : server_(server), socket_(socket) {}

  bool writeAll(const uint8_t* bytes, size_t length) override {
    size_t writtenTotal = 0;
    while (writtenTotal < length) {
      const int written = httpd_socket_send(
          server_, socket_, reinterpret_cast<const char*>(bytes + writtenTotal),
          length - writtenTotal, 0);
      if (written <= 0) {
        return false;
      }
      writtenTotal += static_cast<size_t>(written);
    }
    return true;
  }

 private:
  httpd_handle_t server_;
  int socket_;
};

}  // namespace

HeadHttpsStream::HeadHttpsStream(HeadTelemetry& telemetry,
                                 const char* certificatePem,
                                 const char* privateKeyPem)
    : telemetry_(telemetry),
      streamSession_(telemetry),
      server_(nullptr),
      liveRequest_(nullptr),
      liveSocket_(-1),
      lastServiceQueueAtMs_(0),
      workQueued_(false),
      stateLock_(portMUX_INITIALIZER_UNLOCKED),
      certificatePem_(certificatePem),
      privateKeyPem_(privateKeyPem) {}

void HeadHttpsStream::begin() {
  httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();
  config.httpd.max_open_sockets = 4;  // ESP-IDF reserves three for HTTPD itself.
  config.httpd.max_uri_handlers = 2;
  config.httpd.max_resp_headers = 4;
  config.httpd.max_req_hdr_len = ardb_head::kMaxHttpRequestBytes;
  config.httpd.max_uri_len = 32;
  config.httpd.lru_purge_enable = false;
  config.httpd.recv_wait_timeout = 3;
  config.httpd.send_wait_timeout = 3;
  config.port_secure = ardb_head::kApiPort;
  config.servercert =
      reinterpret_cast<const uint8_t*>(certificatePem_);
  config.servercert_len = strlen(certificatePem_) + 1;
  config.prvtkey_pem =
      reinterpret_cast<const uint8_t*>(privateKeyPem_);
  config.prvtkey_len = strlen(privateKeyPem_) + 1;

  if (httpd_ssl_start(&server_, &config) != ESP_OK) {
    Serial.println("Failed to start ESP32 HTTPS head API.");
    server_ = nullptr;
    return;
  }

  httpd_uri_t liveRoute{};
  liveRoute.uri = ardb_head::kLivePath;
  liveRoute.method = HTTP_GET;
  liveRoute.handler = onLiveRequest;
  liveRoute.user_ctx = this;

  httpd_uri_t healthRoute{};
  healthRoute.uri = ardb_head::kHealthPath;
  healthRoute.method = HTTP_GET;
  healthRoute.handler = onHealthRequest;
  healthRoute.user_ctx = this;

  if (httpd_register_uri_handler(server_, &liveRoute) != ESP_OK ||
      httpd_register_uri_handler(server_, &healthRoute) != ESP_OK) {
    Serial.println("Failed to register ESP32 HTTPS head routes.");
    httpd_ssl_stop(server_);
    server_ = nullptr;
  }
}

void HeadHttpsStream::update() {
  if (server_ == nullptr || !hasLiveStream()) {
    return;
  }

  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&stateLock_);
  const bool due = !workQueued_ &&
                   isDue(nowMs, lastServiceQueueAtMs_ + kServicePollIntervalMs);
  if (due) {
    workQueued_ = true;
    lastServiceQueueAtMs_ = nowMs;
  }
  portEXIT_CRITICAL(&stateLock_);

  if (!due) {
    return;
  }

  if (httpd_queue_work(server_, serviceLiveWork, this) != ESP_OK) {
    portENTER_CRITICAL(&stateLock_);
    workQueued_ = false;
    portEXIT_CRITICAL(&stateLock_);
  }
}

esp_err_t HeadHttpsStream::onLiveRequest(httpd_req_t* request) {
  return static_cast<HeadHttpsStream*>(request->user_ctx)
      ->handleLiveRequest(request);
}

esp_err_t HeadHttpsStream::onHealthRequest(httpd_req_t* request) {
  return static_cast<HeadHttpsStream*>(request->user_ctx)
      ->handleHealthRequest(request);
}

esp_err_t HeadHttpsStream::handleLiveRequest(httpd_req_t* request) {
  if (request->content_len != 0 ||
      httpd_req_get_url_query_len(request) != 0) {
    return sendTextResponse(request, "400 Bad Request", "unsupported request shape\n")
               ? ESP_OK
               : ESP_FAIL;
  }

  if (hasLiveStream()) {
    return sendTextResponse(request, "503 Service Unavailable",
                            "live stream already active\n")
               ? ESP_OK
               : ESP_FAIL;
  }

  httpd_req_t* asyncRequest = nullptr;
  if (httpd_req_async_handler_begin(request, &asyncRequest) != ESP_OK) {
    return ESP_FAIL;
  }

  const int socket = httpd_req_to_sockfd(asyncRequest);
  if (socket < 0) {
    httpd_req_async_handler_complete(asyncRequest);
    return ESP_FAIL;
  }

  portENTER_CRITICAL(&stateLock_);
  liveRequest_ = asyncRequest;
  liveSocket_ = socket;
  workQueued_ = false;
  lastServiceQueueAtMs_ = millis();
  portEXIT_CRITICAL(&stateLock_);

  char responseHeader[320];
  const int responseHeaderLength =
      snprintf(responseHeader, sizeof(responseHeader),
               "HTTP/1.1 200 OK\r\n"
               "Content-Type: application/octet-stream\r\n"
               "Transfer-Encoding: chunked\r\n"
               "Cache-Control: no-store\r\n"
               "Access-Control-Allow-Origin: %s\r\n"
               "X-Content-Type-Options: nosniff\r\n"
               "Connection: keep-alive\r\n\r\n",
               ardb_head::kCorsAllowOrigin);
  if (responseHeaderLength <= 0 ||
      static_cast<size_t>(responseHeaderLength) >= sizeof(responseHeader) ||
      httpd_send(asyncRequest, responseHeader,
                 static_cast<size_t>(responseHeaderLength)) !=
          responseHeaderLength) {
    closeLiveInHttpdTask();
    return ESP_FAIL;
  }

  HttpdRequestWriter writer(*asyncRequest);
  if (!streamSession_.begin(writer, millis())) {
    closeLiveInHttpdTask();
    return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t HeadHttpsStream::handleHealthRequest(httpd_req_t* request) {
  if (request->content_len != 0 ||
      httpd_req_get_url_query_len(request) != 0) {
    return sendTextResponse(request, "400 Bad Request", "unsupported request shape\n")
               ? ESP_OK
               : ESP_FAIL;
  }

  if (hasLiveStream()) {
    return sendTextResponse(request, "503 Service Unavailable",
                            "live stream already active\n")
               ? ESP_OK
               : ESP_FAIL;
  }

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
  if (bodyLength <= 0 || static_cast<size_t>(bodyLength) >= sizeof(body)) {
    return ESP_FAIL;
  }

  if (httpd_resp_set_type(request, HTTPD_TYPE_JSON) != ESP_OK ||
      httpd_resp_set_hdr(request, "Cache-Control", "no-store") != ESP_OK ||
      httpd_resp_set_hdr(request, "Access-Control-Allow-Origin",
                         ardb_head::kCorsAllowOrigin) != ESP_OK ||
      httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff") != ESP_OK ||
      httpd_resp_set_hdr(request, "Connection", "close") != ESP_OK) {
    return ESP_FAIL;
  }
  if (httpd_resp_send(request, body, bodyLength) != ESP_OK) {
    return ESP_FAIL;
  }

  const int socket = httpd_req_to_sockfd(request);
  return socket >= 0 && httpd_sess_trigger_close(server_, socket) == ESP_OK
             ? ESP_OK
             : ESP_FAIL;
}

void HeadHttpsStream::serviceLiveWork(void* context) {
  static_cast<HeadHttpsStream*>(context)->serviceLiveInHttpdTask();
}

void HeadHttpsStream::serviceLiveInHttpdTask() {
  httpd_handle_t server = nullptr;
  int socket = -1;
  portENTER_CRITICAL(&stateLock_);
  workQueued_ = false;
  server = server_;
  socket = liveSocket_;
  portEXIT_CRITICAL(&stateLock_);

  if (server == nullptr || socket < 0) {
    return;
  }

  HttpdSocketWriter writer(server, socket);
  if (!streamSession_.update(writer, millis())) {
    closeLiveInHttpdTask();
  }
}

void HeadHttpsStream::closeLiveInHttpdTask() {
  httpd_req_t* request = nullptr;
  httpd_handle_t server = nullptr;
  int socket = -1;
  portENTER_CRITICAL(&stateLock_);
  request = liveRequest_;
  server = server_;
  socket = liveSocket_;
  liveRequest_ = nullptr;
  liveSocket_ = -1;
  workQueued_ = false;
  portEXIT_CRITICAL(&stateLock_);

  if (request == nullptr) {
    return;
  }

  if (server != nullptr && socket >= 0) {
    httpd_sess_trigger_close(server, socket);
  }
  httpd_req_async_handler_complete(request);
}

bool HeadHttpsStream::hasLiveStream() const {
  portENTER_CRITICAL(&stateLock_);
  const bool hasLive = liveRequest_ != nullptr;
  portEXIT_CRITICAL(&stateLock_);
  return hasLive;
}

bool HeadHttpsStream::sendTextResponse(httpd_req_t* request, const char* status,
                                       const char* body) const {
  return httpd_resp_set_status(request, status) == ESP_OK &&
         httpd_resp_set_type(request, "text/plain; charset=utf-8") == ESP_OK &&
         httpd_resp_set_hdr(request, "Cache-Control", "no-store") == ESP_OK &&
         httpd_resp_set_hdr(request, "Access-Control-Allow-Origin",
                            ardb_head::kCorsAllowOrigin) == ESP_OK &&
         httpd_resp_send(request, body, HTTPD_RESP_USE_STRLEN) == ESP_OK;
}

#endif  // defined(ESP32)
