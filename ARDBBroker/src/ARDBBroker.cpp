#include "ARDBBroker.h"

#include "BrokerNetwork.h"
#include "HeadHttpsStream.h"

namespace ardb_broker {

Runtime::Runtime(const ardb_network::Config& network, const char* certificatePem,
                 const char* privateKeyPem, uint16_t mqttPort,
                 uint8_t retainedMessageCapacity)
    : network_(network),
      certificatePem_(certificatePem),
      privateKeyPem_(privateKeyPem),
      broker_(mqttPort, retainedMessageCapacity),
      telemetry_(broker_),
      headHttps_(telemetry_, certificatePem, privateKeyPem),
      localNetworkValidation_(
          network_.mode == ardb_network::Mode::AccessPoint &&
          ARDB_HEAD_ENABLE_LOCAL_NETWORK_VALIDATION != 0) {}

bool Runtime::begin() {
  if (certificatePem_ == nullptr || privateKeyPem_ == nullptr ||
      certificatePem_[0] == '\0' || privateKeyPem_[0] == '\0') {
    Serial.println("Set certificate and private-key PEM in the sketch.");
    return false;
  }
  if (!beginBrokerNetwork(network_)) {
    return false;
  }

  broker_.begin();
  telemetry_.begin();
  headHttps_.begin();
  localNetworkValidation_.begin();
  return true;
}

void Runtime::update() {
  broker_.loop();
  headHttps_.update();
  localNetworkValidation_.update();
  yield();
}

IPAddress Runtime::address() const {
  return brokerNetworkAddress(network_);
}

const char* Runtime::networkModeName() const {
  return brokerNetworkModeName(network_.mode);
}

MqttBroker& Runtime::mqttBroker() {
  return broker_;
}

const HeadTelemetry& Runtime::telemetry() const {
  return telemetry_;
}

}  // namespace ardb_broker
