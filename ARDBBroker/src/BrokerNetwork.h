#pragma once

#include <Arduino.h>

#include "BrokerNetworkConfig.h"

// Starts the configured AP or joins the configured existing Wi-Fi network.
// It is deliberately the only place that owns WiFi.mode(), so the rest of the
// broker/head code is network-topology independent.
bool beginBrokerNetwork(const ardb_network::Config& config);

// Valid only after beginBrokerNetwork() returns true.
IPAddress brokerNetworkAddress(const ardb_network::Config& config);

const char* brokerNetworkModeName(ardb_network::Mode mode);
