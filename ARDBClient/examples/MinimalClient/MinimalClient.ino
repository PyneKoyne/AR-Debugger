/*
 * ARDB minimal publisher for boards with a WiFi.h-compatible Wi-Fi library.
 *
 * Install in Arduino Library Manager:
 *   - ArduinoMqttClient
 *   - ARDBClient (copy this ARDBClient folder into your Arduino libraries)
 *
 * This example targets the Pico W/Pico 2W Arduino core used by the broker
 * sketch in this workspace. For other boards, replace WiFi.h and the two
 * Wi-Fi callbacks with that board's equivalent API.
 */

#include <WiFi.h>
#include <ARDBClient.h>

const char* WIFI_SSID = "Broker1";
const char* WIFI_PASSWORD = "abcdefghi";
const char* MQTT_HOST = "192.168.4.1";  // MQTT broker IP, not the Wi-Fi SSID
const uint16_t MQTT_PORT = 1883;

WiFiClient network;

void beginArdbNetwork(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

bool ardbNetworkConnected() {
  return WiFi.status() == WL_CONNECTED;
}

ARDBNetworkCallbacks ardbNetwork(beginArdbNetwork, ardbNetworkConnected);

// The factory keeps common settings on one line. Port, retry interval, and
// enabled state are optional trailing arguments when their defaults do not fit.
ARDBConfig ardbConfig = ARDBConfig::wifiMqtt(
    WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, "demo-pico-01", MQTT_PORT);
// Each topic is published at most 10 Hz by default; pass a fourth constructor
// argument here to override it, for example /* dataPublishRateHz */ 25.
ARDBClient ardb(network, ardbNetwork, ardbConfig);
ARDBTopic imuTopic;
ARDBTopic temperatureTopic;
ARDBTopic logTopic;

struct __attribute__((packed)) ImuFrame {
  uint32_t timeMs;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

void setup() {
  Serial.begin(115200);

  // The callbacks remain explicit and board-specific, while ardb itself is
  // an object so calls use familiar dot syntax.
  imuTopic = ardb.addTopic("a/demo/i", ARDBVisualType::Imu6I16T32,
                           "Demo IMU", 16);
  temperatureTopic = ardb.addTopic("a/demo/t", ARDBVisualType::ScalarF32,
                                    "Demo temperature", 4);
  logTopic = ardb.addTopic("a/demo/l", ARDBVisualType::Log, "Demo log");
  ardb.begin();
}

void loop() {
  ardb.update();  // never loops until connected; call every iteration

  static bool connectionReported = false;
  if (ardb.connected() && !connectionReported) {
    ardb.print(logTopic, "ARDB connected");  // text length is inferred
    connectionReported = true;
  } else if (!ardb.connected()) {
    connectionReported = false;
  }

  static uint32_t lastSampleAt = 0;
  if (millis() - lastSampleAt >= 50) {
    lastSampleAt = millis();
    const ImuFrame frame = {static_cast<uint32_t>(millis()), 100, 0, 1024, 0, 0, 0};

    // The typed overload infers sizeof(frame) and sizeof(float). Both calls
    // are no-ops that return false, and increment droppedPackets(), offline.
    ardb.print(imuTopic, frame);

    const float temperatureC = 21.5f;
    ardb.print(temperatureTopic, temperatureC);
  }
}
