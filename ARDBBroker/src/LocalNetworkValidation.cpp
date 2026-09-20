#include "LocalNetworkValidation.h"

#include <string.h>

namespace {

constexpr uint16_t kDnsPort = 53;
constexpr uint16_t kHttpPort = 80;
constexpr size_t kMaxDnsPacketBytes = 384;
constexpr uint32_t kHttpRequestTimeoutMs = 1000;
constexpr uint8_t kHttpReadBudgetBytes = 96;

const char kNoContentResponse[] =
    "HTTP/1.1 204 No Content\r\n"
    "Content-Length: 0\r\n"
    "Cache-Control: no-store\r\n"
    "Connection: close\r\n\r\n";

bool isDue(uint32_t nowMs, uint32_t targetMs) {
  return static_cast<int32_t>(nowMs - targetMs) >= 0;
}

uint16_t readBigEndianU16(const uint8_t* bytes) {
  return static_cast<uint16_t>(bytes[0]) << 8 | bytes[1];
}

void writeBigEndianU16(uint8_t* bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value >> 8);
  bytes[1] = static_cast<uint8_t>(value & 0xff);
}

}  // namespace

LocalNetworkValidation::LocalNetworkValidation(bool enabled)
    : enabled_(enabled),
      started_(false),
      dnsServer_(),
      httpServer_(kHttpPort),
      httpClient_(),
      httpRequestDeadlineMs_(0) {}

void LocalNetworkValidation::begin() {
  if (!enabled_ || started_) {
    return;
  }

  if (dnsServer_.begin(kDnsPort) == 0) {
    Serial.println("Local network validation DNS could not start.");
    return;
  }

  httpServer_.begin();
  started_ = true;
  Serial.println("Local network validation responder ready.");
}

void LocalNetworkValidation::update() {
  if (!started_) {
    return;
  }

  answerDnsQuery();
  serviceHttpProbe();
}

void LocalNetworkValidation::answerDnsQuery() {
  const int packetLength = dnsServer_.parsePacket();
  if (packetLength <= 0) {
    return;
  }

  const IPAddress requesterAddress = dnsServer_.remoteIP();
  const uint16_t requesterPort = dnsServer_.remotePort();
  uint8_t query[kMaxDnsPacketBytes];
  const size_t bytesToRead = packetLength > static_cast<int>(sizeof(query))
                                 ? sizeof(query)
                                 : static_cast<size_t>(packetLength);
  const int bytesRead = dnsServer_.read(query, bytesToRead);
  while (dnsServer_.available() > 0) {
    dnsServer_.read();
  }

  if (bytesRead < 0 || static_cast<size_t>(bytesRead) != bytesToRead ||
      packetLength != bytesRead || bytesToRead < 12 ||
      (query[2] & 0x80) != 0 || readBigEndianU16(query + 4) != 1) {
    return;
  }

  size_t questionEnd = 12;
  while (questionEnd < bytesToRead) {
    const uint8_t labelLength = query[questionEnd++];
    if (labelLength == 0) {
      break;
    }
    if (labelLength > 63 || questionEnd + labelLength > bytesToRead) {
      return;
    }
    questionEnd += labelLength;
  }

  if (questionEnd + 4 > bytesToRead ||
      readBigEndianU16(query + questionEnd) != 1 ||
      readBigEndianU16(query + questionEnd + 2) != 1) {
    return;
  }
  questionEnd += 4;

  uint8_t response[kMaxDnsPacketBytes + 16];
  memcpy(response, query, questionEnd);
  response[2] = 0x81;
  response[3] = 0x80;
  writeBigEndianU16(response + 4, 1);
  writeBigEndianU16(response + 6, 1);
  writeBigEndianU16(response + 8, 0);
  writeBigEndianU16(response + 10, 0);

  size_t responseLength = questionEnd;
  response[responseLength++] = 0xc0;
  response[responseLength++] = 0x0c;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x01;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x01;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x3c;
  response[responseLength++] = 0x00;
  response[responseLength++] = 0x04;

  const IPAddress accessPointAddress = WiFi.softAPIP();
  for (uint8_t octet = 0; octet < 4; ++octet) {
    response[responseLength++] = accessPointAddress[octet];
  }

  if (dnsServer_.beginPacket(requesterAddress, requesterPort) != 0) {
    dnsServer_.write(response, responseLength);
    dnsServer_.endPacket();
  }
}

void LocalNetworkValidation::serviceHttpProbe() {
  if (!httpClient_) {
    WiFiClient incoming = httpServer_.accept();
    if (!incoming) {
      return;
    }
    httpClient_ = incoming;
    httpRequestDeadlineMs_ = millis() + kHttpRequestTimeoutMs;
  }

  if (!httpClient_.connected()) {
    closeHttpClient();
    return;
  }

  if (httpClient_.available() == 0 &&
      !isDue(millis(), httpRequestDeadlineMs_)) {
    return;
  }

  uint8_t consumed = 0;
  while (httpClient_.available() > 0 && consumed < kHttpReadBudgetBytes) {
    if (httpClient_.read() < 0) {
      closeHttpClient();
      return;
    }
    ++consumed;
  }

  httpClient_.write(reinterpret_cast<const uint8_t*>(kNoContentResponse),
                    sizeof(kNoContentResponse) - 1);
  closeHttpClient();
}

void LocalNetworkValidation::closeHttpClient() {
  httpClient_.stop();
  httpClient_ = WiFiClient();
  httpRequestDeadlineMs_ = 0;
}
