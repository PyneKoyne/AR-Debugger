/*
 * Raspberry Pi Pico 2 W TMC6300 BLE -> ESP32 -> ARDB MQTT bridge.
 *
 * The companion Pico sketch advertises one BLE notify characteristic carrying
 * three 32-bit floats: the UH, VH, and WH PWM duties. This ESP32 connects to
 * that characteristic and republishes the unchanged 12-byte payload as the
 * original ARDB THREE_NUM stream on b/demo/a.
 *
 * Board: any ESP32 with native BLE and Wi-Fi support.
 * Libraries: built-in ESP32 BLE and WiFi, ArduinoMqttClient, ARDBClient.
 *
 * Copy arduino_secrets.h.example to arduino_secrets.h before compiling.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <map>
#include <string.h>
#include <string>

#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

#include <ARDBClient.h>

#include "arduino_secrets.h"

namespace {

constexpr char kClientId[] = "esp32-pico-tmc6300-bridge";
constexpr char kPicoServiceUuid[] = "a148b750-92c6-4f9a-9f61-a9f1d376adc1";
constexpr char kPicoPwmCharacteristicUuid[] =
    "a148b751-92c6-4f9a-9f61-a9f1d376adc1";
constexpr char kPicoDeviceName[] = "ARDB-PWM";
constexpr char kPwmTopicPath[] = "b/demo/a";
constexpr size_t kPwmPayloadBytes = 3 * sizeof(float);
constexpr uint32_t kBleScanIntervalMs = 5000;
constexpr uint16_t kWifiConnectTimeoutMs = 30000;

static_assert(sizeof(float) == 4,
              "The Pico sends three 32-bit IEEE-754 float values.");

struct PwmPayload {
  uint8_t bytes[kPwmPayloadBytes];
};

WiFiClient mqttTransport;
bool wifiConfigured = false;

void beginWifi(const char* ssid, const char* password) {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED || status == WL_IDLE_STATUS) {
    // WL_IDLE_STATUS means the station has associated and is still obtaining
    // an IP address. Calling WiFi.begin() here causes ESP-IDF's "sta is
    // connecting, cannot set config" error and restarts that work.
    return;
  }

  if (!wifiConfigured) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    wifiConfigured = true;
  }

  // ESP32's Wi-Fi driver reconnects automatically. Do not call WiFi.begin()
  // or WiFi.reconnect() again here: either call can interrupt an association
  // still in progress and emits "sta is connecting" errors.
}

bool wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

ARDBConfig makeArdbConfig() {
  ARDBConfig config = ARDBConfig::wifiMqtt(
      SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD, SECRET_MQTT_HOST, kClientId,
      SECRET_MQTT_PORT, /*retrySeconds=*/5, /*enabled=*/true);
  config.wifiConnectTimeoutMs = kWifiConnectTimeoutMs;
  return config;
}

ARDBConfig ardbConfig = makeArdbConfig();
ARDBNetworkCallbacks ardbNetwork(beginWifi, wifiConnected);
ARDBClient ardb(mqttTransport, ardbNetwork, ardbConfig);
ARDBTopic pwmTopic;

QueueHandle_t receivedPwm = nullptr;
portMUX_TYPE bleStateLock = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t receivedPwmCount = 0;
uint32_t publishedPwmCount = 0;

BLEScan* scanner = nullptr;
BLEClient* picoClient = nullptr;
BLEAdvertisedDevice* pendingDevice = nullptr;
bool bleConnected = false;
bool scanInProgress = false;
uint32_t nextScanAt = 0;

void onPicoPwmNotification(BLERemoteCharacteristic*, uint8_t* data,
                           size_t length, bool) {
  // A complete original ARDB THREE_NUM sample is exactly three floats. Never
  // forward an unexpected characteristic value as motor telemetry.
  if (data == nullptr || length != kPwmPayloadBytes || receivedPwm == nullptr) {
    return;
  }

  PwmPayload payload{};
  memcpy(payload.bytes, data, sizeof(payload.bytes));
  ++receivedPwmCount;

  // The only queued value is replaced so the broker receives the newest PWM
  // sample after a short Wi-Fi/MQTT outage rather than a stale backlog.
  xQueueOverwrite(receivedPwm, &payload);
}

const char* wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED:
      return "connected";
    case WL_IDLE_STATUS:
      return "joining/DHCP";
    case WL_NO_SSID_AVAIL:
      return "SSID not found";
    case WL_CONNECT_FAILED:
      return "authentication failed";
    case WL_CONNECTION_LOST:
      return "connection lost";
    case WL_DISCONNECTED:
      return "disconnected";
    case WL_STOPPED:
      return "stopped";
    default:
      return "unknown";
  }
}

class PicoClientCallbacks final : public BLEClientCallbacks {
 public:
  void onDisconnect(BLEClient*) override {
    portENTER_CRITICAL(&bleStateLock);
    bleConnected = false;
    nextScanAt = millis() + kBleScanIntervalMs;
    portEXIT_CRITICAL(&bleStateLock);
    Serial.println("[ble] Pico disconnected");
  }
};

void onScanComplete(BLEScanResults) {
  portENTER_CRITICAL(&bleStateLock);
  scanInProgress = false;
  if (pendingDevice == nullptr && !bleConnected) {
    nextScanAt = millis() + kBleScanIntervalMs;
  }
  portEXIT_CRITICAL(&bleStateLock);
}

class PicoAdvertisementCallbacks final : public BLEAdvertisedDeviceCallbacks {
 public:
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    const bool hasExpectedService =
        advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(kPicoServiceUuid));
    const bool hasExpectedName =
        advertisedDevice.haveName() &&
        strcmp(advertisedDevice.getName().c_str(), kPicoDeviceName) == 0;
    if (!hasExpectedService && !hasExpectedName) {
      return;
    }

    BLEAdvertisedDevice* candidate = new BLEAdvertisedDevice(advertisedDevice);
    bool accepted = false;
    portENTER_CRITICAL(&bleStateLock);
    if (pendingDevice == nullptr && !bleConnected) {
      pendingDevice = candidate;
      candidate = nullptr;
      scanInProgress = false;
      accepted = true;
    }
    portEXIT_CRITICAL(&bleStateLock);

    delete candidate;
    if (accepted) {
      Serial.println(hasExpectedService
                         ? "[ble] found Pico PWM service"
                         : "[ble] found Pico by name; checking its service");
      scanner->stop();
    }
  }
};

bool connectToPico(BLEAdvertisedDevice* device) {
  if (device == nullptr) {
    return false;
  }

  if (picoClient == nullptr) {
    picoClient = BLEDevice::createClient();
    if (picoClient == nullptr) {
      return false;
    }
    picoClient->setClientCallbacks(new PicoClientCallbacks());
  }

  if (!picoClient->connect(device)) {
    Serial.println("[ble] Pico connection attempt failed");
    return false;
  }

  // The 12-byte payload works at the default 23-byte ATT MTU. A larger MTU
  // is not required, so this bridge remains compatible with the Pico default.
  BLERemoteService* service = picoClient->getService(BLEUUID(kPicoServiceUuid));
  BLERemoteCharacteristic* characteristic = nullptr;
  if (service != nullptr) {
    characteristic =
        service->getCharacteristic(BLEUUID(kPicoPwmCharacteristicUuid));
  }

  if (characteristic == nullptr || !characteristic->canNotify()) {
    // The ESP32 has not matched the Pico service UUID during discovery. We
    // already require the exact ARDB-PWM device name, then select only a
    // notification characteristic; the callback still rejects every payload
    // except the required 12-byte PWM sample.
    Serial.println("[ble] locating Pico PWM notification characteristic");
    characteristic = nullptr;
    std::map<std::string, BLERemoteService*>* services = picoClient->getServices();
    for (const auto& serviceEntry : *services) {
      BLERemoteService* candidateService = serviceEntry.second;
      if (candidateService == nullptr) {
        continue;
      }
      std::map<std::string, BLERemoteCharacteristic*>* characteristics =
          candidateService->getCharacteristics();
      for (const auto& characteristicEntry : *characteristics) {
        BLERemoteCharacteristic* candidate = characteristicEntry.second;
        if (candidate != nullptr && candidate->canNotify()) {
          characteristic = candidate;
          Serial.print("[ble] using Pico notify characteristic ");
          Serial.println(candidate->getUUID().toString().c_str());
          break;
        }
      }
      if (characteristic != nullptr) {
        break;
      }
    }
  }

  if (characteristic == nullptr || !characteristic->canNotify()) {
    Serial.println("[ble] Pico has no usable PWM notification characteristic");
    picoClient->disconnect();
    return false;
  }

  characteristic->registerForNotify(onPicoPwmNotification);
  portENTER_CRITICAL(&bleStateLock);
  bleConnected = true;
  portEXIT_CRITICAL(&bleStateLock);
  Serial.println("[ble] connected and subscribed to Pico PWM");
  return true;
}

void serviceBle() {
  const uint32_t now = millis();

  BLEAdvertisedDevice* device = nullptr;
  portENTER_CRITICAL(&bleStateLock);
  device = pendingDevice;
  pendingDevice = nullptr;
  portEXIT_CRITICAL(&bleStateLock);
  if (device != nullptr) {
    const bool connected = connectToPico(device);
    delete device;
    if (!connected) {
      Serial.println("[ble] connection/subscription failed");
      portENTER_CRITICAL(&bleStateLock);
      nextScanAt = millis() + kBleScanIntervalMs;
      portEXIT_CRITICAL(&bleStateLock);
    }
    return;
  }

  bool beginScan = false;
  portENTER_CRITICAL(&bleStateLock);
  if (!bleConnected && !scanInProgress &&
      static_cast<int32_t>(now - nextScanAt) >= 0) {
    scanInProgress = true;
    beginScan = true;
  }
  portEXIT_CRITICAL(&bleStateLock);
  if (beginScan) {
    Serial.println("[ble] scanning for ARDB-PWM");
    const bool started = scanner->start(/*durationSeconds=*/4, onScanComplete,
                                        /*isContinue=*/false);
    if (!started) {
      portENTER_CRITICAL(&bleStateLock);
      scanInProgress = false;
      nextScanAt = millis() + kBleScanIntervalMs;
      portEXIT_CRITICAL(&bleStateLock);
    }
  }
}

void publishPwm() {
  if (!ardb.connected() || receivedPwm == nullptr) {
    return;
  }

  PwmPayload payload{};
  if (xQueuePeek(receivedPwm, &payload, 0) != pdTRUE) {
    return;
  }

  // ARDBClient enforces its regular publishing rate. Keep this newest queued
  // sample until it is accepted by the MQTT transport.
  if (ardb.printBytes(pwmTopic, payload.bytes, sizeof(payload.bytes))) {
    xQueueReceive(receivedPwm, &payload, 0);
    ++publishedPwmCount;
  }
}

void reportBridgeStatus() {
  static uint32_t lastReportedAtMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastReportedAtMs < 2000) {
    return;
  }
  lastReportedAtMs = nowMs;

  Serial.print("[status] Wi-Fi=");
  Serial.print(wifiStatusName(WiFi.status()));
  Serial.print(" MQTT=");
  Serial.print(ardb.connected() ? "connected" : "waiting");
  Serial.print(" BLE=");
  Serial.print(bleConnected ? "connected" : "scanning");
  Serial.print(" BLE samples=");
  Serial.print(receivedPwmCount);
  Serial.print(" MQTT samples=");
  Serial.println(publishedPwmCount);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("[boot] starting Pico TMC6300 BLE-to-ARDB bridge");

  receivedPwm = xQueueCreate(/*queueLength=*/1, sizeof(PwmPayload));
  if (receivedPwm == nullptr) {
    Serial.println("[boot] FATAL: PWM queue allocation failed");
    return;
  }

  pwmTopic = ardb.addTopic(kPwmTopicPath, ARDBVisualType::THREE_NUM,
                           "Demo BLDC PWM", kPwmPayloadBytes);
  ardb.begin();

  Serial.println("[boot] starting BLE scanner");
  BLEDevice::init("ardb-pico-tmc6300");
  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new PicoAdvertisementCallbacks());
  scanner->setActiveScan(true);
  scanner->setInterval(160);
  scanner->setWindow(80);
  nextScanAt = millis();

  Serial.println("[boot] Pico TMC6300 BLE-to-ARDB bridge ready");
}

void loop() {
  ardb.update();
  serviceBle();
  publishPwm();
  reportBridgeStatus();
  delay(2);
}
