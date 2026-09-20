#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// A tiny access-point-only compatibility service for devices that otherwise
// discard a local Wi-Fi network after their HTTP connectivity check fails. It
// does not route or proxy traffic to the Internet.
class LocalNetworkValidation final {
 public:
  explicit LocalNetworkValidation(bool enabled);

  void begin();
  void update();

 private:
  void answerDnsQuery();
  void serviceHttpProbe();
  void closeHttpClient();

  bool enabled_;
  bool started_;
  WiFiUDP dnsServer_;
  WiFiServer httpServer_;
  WiFiClient httpClient_;
  uint32_t httpRequestDeadlineMs_;
};
