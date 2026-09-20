#pragma once

#include <Arduino.h>

#include "BrokerNetworkConfig.h"
#include "HeadConfig.h"
#include "HeadHttpsStream.h"
#include "HeadTelemetry.h"
#include "TinyMqtt.h"

namespace ardb_broker {

// Application-facing façade for the embedded MQTT broker and HTTPS head node.
// Credentials, certificate PEM, and network mode are intentionally supplied by
// the sketch, never compiled into the reusable library.
class Runtime final {
 public:
  Runtime(const ardb_network::Config& network, const char* certificatePem,
          const char* privateKeyPem,
          uint16_t mqttPort = ardb_network::kMqttPort,
          uint8_t retainedMessageCapacity =
              static_cast<uint8_t>(ardb_head::kMaxStreams));

  // Starts networking, then MQTT and HTTPS. Returns false only when network or
  // credential configuration is invalid; callers choose their own retry/LED
  // policy in the sketch.
  bool begin();

  // Cooperative service call; invoke once from loop().
  void update();

  IPAddress address() const;
  const char* networkModeName() const;
  MqttBroker& mqttBroker();
  const HeadTelemetry& telemetry() const;

 private:
  // Keep a small copy so a caller may construct Runtime from a local Config.
  // The SSID and password pointers themselves must remain valid for Runtime's
  // lifetime, as they normally do when backed by sketch string literals.
  ardb_network::Config network_;
  const char* certificatePem_;
  const char* privateKeyPem_;
  MqttBroker broker_;
  HeadTelemetry telemetry_;
  HeadHttpsStream headHttps_;
};

}  // namespace ardb_broker
