#pragma once

#include <Arduino.h>
#include <vector>

#include "TinyMqtt.h"

class MqttEventLogger {
 public:
  explicit MqttEventLogger(MqttBroker& broker,
                           const char* loggerId = "broker-logger");

  // Registers the local monitor client. Call once after Serial.begin().
  void begin();

  // Prints client connection changes. Call once per loop after broker.loop().
  void update();

 private:
  struct SeenClient {
    const MqttClient* client;
    String id;
  };

  static void onMqttPublish(const MqttClient* source, const Topic& topic,
                            const char* payload, size_t payloadLength);
  void logMqttPublish(const Topic& topic, const char* payload,
                      size_t payloadLength);
  void logClientEvents();
  static void printEscapedPayload(const char* payload, size_t length);

  MqttBroker& broker_;
  MqttClient loggerClient_;
  std::vector<SeenClient> seenClients_;

  // TinyMqtt callbacks do not provide a user-data pointer, so this module
  // supports one logger instance per sketch.
  static MqttEventLogger* instance_;
};
