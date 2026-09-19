/*
 * MQTT Broker for Wifi Enabled Dev Boards
*/

#include <WiFi.h>
#include "TinyMqtt.h"   // https://github.com/hsaturn/TinyMqtt >= 1.1.0 (manual install, see below)
#include "MqttEventLogger.h"

const uint16_t PORT = 1883;
const uint8_t  RETAIN = 10;  // Max retained messages

MqttBroker broker(PORT, RETAIN);
MqttEventLogger mqttLogger(broker);

  const char* ssid = "Broker1";
  const char* password = "abcdefghi";

static void waitForSerial(unsigned long timeout_ms = 3000)
{
  unsigned long start = millis();
  while (!Serial && (millis() - start < timeout_ms)) {
    delay(10);
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  waitForSerial();

  Serial.println();
  Serial.println("Pico 2W MQTT Broker (TinyMqtt)");

  if (strlen(ssid) == 0) {
    Serial.println("****** PLEASE MODIFY ssid/password *************");
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(250);
    }
  }

  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);

  // Start the AP
  bool res = WiFi.softAP(ssid, password);
  
  if (res) {
    Serial.println("Access Point started successfully!");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("SSID: ");
    Serial.println(ssid);
  } else {
    Serial.println("Failed to start Access Point.");
  }

  digitalWrite(LED_BUILTIN, HIGH);  // solid ON once connected (CYW43 LED)

  mqttLogger.begin();

  broker.begin();
  Serial.print("Broker ready: ");
  Serial.print(WiFi.softAPIP());
  Serial.print(" on port ");
  Serial.println(PORT);
}

void loop()
{
  broker.loop();
  mqttLogger.update();
  yield();  // let CYW43/LWIP background tasks run
}
