#include "BrokerNetwork.h"

#include <WiFi.h>

namespace {

bool credentialsArePresent(const ardb_network::Config& config) {
  return config.ssid != nullptr && config.password != nullptr &&
         config.ssid[0] != '\0' && config.password[0] != '\0';
}

IPAddress makeAddress(const uint8_t octets[4]) {
  return IPAddress(octets[0], octets[1], octets[2], octets[3]);
}

bool startAccessPoint(const ardb_network::Config& config) {
  const ardb_network::AccessPointAddress& address = config.accessPointAddress;
  const IPAddress localAddress = makeAddress(address.address);

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(localAddress, makeAddress(address.gateway),
                         makeAddress(address.subnet))) {
    Serial.println("Failed to configure broker access point.");
    return false;
  }

  if (!WiFi.softAP(config.ssid, config.password)) {
    Serial.println("Failed to start broker access point.");
    return false;
  }
  return true;
}

bool joinStationNetwork(const ardb_network::Config& config) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.password);

  // Preserve the original development behavior: do not launch the broker or
  // head API until the device has a usable station association and address.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  return true;
}

}  // namespace

bool beginBrokerNetwork(const ardb_network::Config& config) {
  if (!credentialsArePresent(config)) {
    Serial.println("Set Wi-Fi credentials in the sketch configuration.");
    return false;
  }

  return config.mode == ardb_network::Mode::AccessPoint
             ? startAccessPoint(config)
             : joinStationNetwork(config);
}

IPAddress brokerNetworkAddress(const ardb_network::Config& config) {
  return config.mode == ardb_network::Mode::AccessPoint
             ? WiFi.softAPIP()
             : WiFi.localIP();
}

const char* brokerNetworkModeName(ardb_network::Mode mode) {
  return mode == ardb_network::Mode::AccessPoint
             ? "access point"
             : "station";
}
