#pragma once

#include <Arduino.h>
#include <esp_http_server.h>

#include "HeadConfig.h"
#include "HeadStreamSession.h"
#include "HeadTelemetry.h"

// ESP32 Arduino transport adapter. It uses the ESP-IDF HTTPS server supplied
// by the Arduino-ESP32 core rather than assuming Arduino-Pico BearSSL types.
class HeadHttpsStream final {
 public:
  HeadHttpsStream(HeadTelemetry& telemetry, const char* certificatePem,
                  const char* privateKeyPem);

  void begin();
  void update();

 private:
  static esp_err_t onLiveRequest(httpd_req_t* request);
  static esp_err_t onHealthRequest(httpd_req_t* request);
  static void serviceLiveWork(void* context);

  esp_err_t handleLiveRequest(httpd_req_t* request);
  esp_err_t handleHealthRequest(httpd_req_t* request);
  void serviceLiveInHttpdTask();
  void closeLiveInHttpdTask();
  bool hasLiveStream() const;
  bool sendTextResponse(httpd_req_t* request, const char* status,
                        const char* body) const;

  HeadTelemetry& telemetry_;
  HeadStreamSession streamSession_;
  httpd_handle_t server_;
  httpd_req_t* liveRequest_;
  int liveSocket_;
  uint32_t lastServiceQueueAtMs_;
  bool workQueued_;
  mutable portMUX_TYPE stateLock_;
  const char* certificatePem_;
  const char* privateKeyPem_;
};
