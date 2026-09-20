#pragma once

#include <Arduino.h>

namespace ardb_network {

enum class Mode : uint8_t {
  AccessPoint,
  Station,
};

struct AccessPointAddress {
  uint8_t address[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
};

constexpr uint16_t kMqttPort = 1883;
// The default access-point address is deliberately kept stable because it is
// part of the development URL and certificate example. It is used only in AP
// mode; station mode obtains its address from the existing network.
constexpr AccessPointAddress kDefaultAccessPointAddress = {
    {192, 168, 4, 1},
    {192, 168, 4, 1},
    {255, 255, 255, 0},
};

// This is supplied by the sketch, so library code never contains Wi-Fi
// credentials or chooses a deployment topology on the caller's behalf.
struct Config {
  Mode mode;
  const char* ssid;
  const char* password;
  AccessPointAddress accessPointAddress;
};

}  // namespace ardb_network
