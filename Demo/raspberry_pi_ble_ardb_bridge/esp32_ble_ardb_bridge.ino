/*
 * Raspberry Pi BLE -> ESP32 -> ARDB MQTT bridge.
 *
 * The Raspberry Pi advertises the service and sends framed notifications. This
 * ESP32 connects as the BLE central, reassembles one application message of at
 * most 64 bytes, and publishes it on a/pi/ble using ARDBClient over Wi-Fi.
 *
 * Board: any ESP32 with native BLE and Wi-Fi support.
 * Libraries: built-in ESP32 BLE, WiFi, ArduinoMqttClient, ARDBClient.
 *
 * Copy arduino_secrets.h.example to arduino_secrets.h before compiling.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>

#include <ARDBClient.h>

#include "arduino_secrets.h"

namespace {

constexpr char kClientId[] = "esp32-pi-ble-bridge";
constexpr char kPiServiceUuid[] = "e7e1b8a5-7d13-44e7-9e85-7a5638d8a100";
constexpr char kPiDataCharacteristicUuid[] =
    "e7e1b8a5-7d13-44e7-9e85-7a5638d8a101";

// Each BLE notification carries this four-byte header plus up to 16 payload
// bytes, so it works even at the default 23-byte ATT MTU.
constexpr uint8_t kBleProtocolVersion = 0xa1;
constexpr uint8_t kBleFrameStart = 0x01;
constexpr uint8_t kBleFrameEnd = 0x02;
constexpr size_t kBleFrameHeaderBytes = 4;
constexpr size_t kMaxApplicationBytes = ARDB_MAX_APPLICATION_PAYLOAD_BYTES;
constexpr uint32_t kBleScanIntervalMs = 5000;
constexpr uint32_t kBleAssemblyTimeoutMs = 1000;

struct BleMessage {
  uint8_t bytes[kMaxApplicationBytes];
  uint8_t length;
};

WiFiClient mqttTransport;

void beginWifi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

bool wifiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

ARDBConfig ardbConfig = ARDBConfig::wifiMqtt(
    SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD, SECRET_MQTT_HOST, kClientId,
    SECRET_MQTT_PORT, /*retrySeconds=*/5, /*enabled=*/true);
ARDBNetworkCallbacks ardbNetwork(beginWifi, wifiConnected);
ARDBClient ardb(mqttTransport, ardbNetwork, ardbConfig);
ARDBTopic piBleTopic;

QueueHandle_t receivedMessages = nullptr;
portMUX_TYPE receiveLock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE bleStateLock = portMUX_INITIALIZER_UNLOCKED;

uint8_t assemblyBytes[kMaxApplicationBytes];
uint8_t assemblyLength = 0;
uint8_t assemblyExpectedLength = 0;
uint8_t assemblySequence = 0;
uint32_t assemblyStartedAt = 0;
bool assemblyActive = false;

BLEScan* scanner = nullptr;
BLEClient* piClient = nullptr;
BLEAdvertisedDevice* pendingDevice = nullptr;
bool bleConnected = false;
bool scanInProgress = false;
uint32_t nextScanAt = 0;

void resetAssembly() {
  portENTER_CRITICAL(&receiveLock);
  assemblyActive = false;
  assemblyLength = 0;
  assemblyExpectedLength = 0;
  portEXIT_CRITICAL(&receiveLock);
}

void onPiNotification(BLERemoteCharacteristic*, uint8_t* data, size_t length,
                      bool) {
  if (data == nullptr || length < kBleFrameHeaderBytes ||
      data[0] != kBleProtocolVersion) {
    return;
  }

  const uint8_t sequence = data[1];
  const uint8_t flags = data[2];
  const uint8_t declaredLength = data[3];
  const uint8_t* const fragment = data + kBleFrameHeaderBytes;
  const size_t fragmentLength = length - kBleFrameHeaderBytes;

  BleMessage completed{};
  bool hasCompletedMessage = false;

  portENTER_CRITICAL(&receiveLock);
  if ((flags & kBleFrameStart) != 0) {
    assemblyActive = declaredLength <= kMaxApplicationBytes;
    assemblySequence = sequence;
    assemblyExpectedLength = declaredLength;
    assemblyLength = 0;
    assemblyStartedAt = millis();
  }

  if (!assemblyActive || sequence != assemblySequence ||
      assemblyLength > assemblyExpectedLength ||
      fragmentLength > static_cast<size_t>(assemblyExpectedLength - assemblyLength)) {
    assemblyActive = false;
  } else {
    if (fragmentLength != 0) {
      memcpy(&assemblyBytes[assemblyLength], fragment, fragmentLength);
      assemblyLength += static_cast<uint8_t>(fragmentLength);
    }

    if ((flags & kBleFrameEnd) != 0) {
      if (assemblyLength == assemblyExpectedLength) {
        completed.length = assemblyLength;
        if (assemblyLength != 0) {
          memcpy(completed.bytes, assemblyBytes, assemblyLength);
        }
        hasCompletedMessage = true;
      }
      assemblyActive = false;
    }
  }
  portEXIT_CRITICAL(&receiveLock);

  // The BLE callback must not call ARDB/MQTT. Queue the completed message for
  // loop(), which handles the network transport cooperatively.
  if (hasCompletedMessage && receivedMessages != nullptr) {
    xQueueSend(receivedMessages, &completed, 0);
  }
}

class PiClientCallbacks final : public BLEClientCallbacks {
 public:
  void onDisconnect(BLEClient*) override {
    portENTER_CRITICAL(&bleStateLock);
    bleConnected = false;
    nextScanAt = millis() + kBleScanIntervalMs;
    portEXIT_CRITICAL(&bleStateLock);
    resetAssembly();
    Serial.println("[ble] Raspberry Pi disconnected");
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

class PiAdvertisementCallbacks final : public BLEAdvertisedDeviceCallbacks {
 public:
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveServiceUUID() ||
        !advertisedDevice.isAdvertisingService(BLEUUID(kPiServiceUuid))) {
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

    if (candidate != nullptr) {
      delete candidate;
    }
    if (accepted) {
      scanner->stop();
    }
  }
};

bool connectToPi(BLEAdvertisedDevice* device) {
  if (device == nullptr) {
    return false;
  }

  if (piClient == nullptr) {
    piClient = BLEDevice::createClient();
    if (piClient == nullptr) {
      return false;
    }
    piClient->setClientCallbacks(new PiClientCallbacks());
  }

  if (!piClient->connect(device)) {
    return false;
  }

  // The protocol also works at the default MTU; requesting a larger MTU just
  // reduces notification overhead when BlueZ permits it.
  piClient->setMTU(100);
  BLERemoteService* service = piClient->getService(BLEUUID(kPiServiceUuid));
  if (service == nullptr) {
    piClient->disconnect();
    return false;
  }

  BLERemoteCharacteristic* characteristic =
      service->getCharacteristic(BLEUUID(kPiDataCharacteristicUuid));
  if (characteristic == nullptr || !characteristic->canNotify()) {
    piClient->disconnect();
    return false;
  }

  characteristic->registerForNotify(onPiNotification);
  portENTER_CRITICAL(&bleStateLock);
  bleConnected = true;
  portEXIT_CRITICAL(&bleStateLock);
  Serial.println("[ble] connected and subscribed to Raspberry Pi data");
  return true;
}

void serviceBle() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&receiveLock);
  if (assemblyActive && now - assemblyStartedAt >= kBleAssemblyTimeoutMs) {
    assemblyActive = false;
    assemblyLength = 0;
    assemblyExpectedLength = 0;
  }
  portEXIT_CRITICAL(&receiveLock);

  BLEAdvertisedDevice* device = nullptr;
  portENTER_CRITICAL(&bleStateLock);
  device = pendingDevice;
  pendingDevice = nullptr;
  portEXIT_CRITICAL(&bleStateLock);
  if (device != nullptr) {
    const bool connected = connectToPi(device);
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

void publishReceivedMessages() {
  if (!ardb.connected() || receivedMessages == nullptr) {
    return;
  }

  BleMessage message{};
  while (xQueuePeek(receivedMessages, &message, 0) == pdTRUE) {
    // ARDBClient applies its normal data-rate limit. Retain the queued item
    // until a publish succeeds, so a temporary MQTT outage does not silently
    // discard the newest messages before the small queue fills.
    if (!ardb.printBytes(piBleTopic, message.bytes, message.length)) {
      return;
    }
    xQueueReceive(receivedMessages, &message, 0);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  receivedMessages = xQueueCreate(/*queueLength=*/4, sizeof(BleMessage));
  if (receivedMessages == nullptr) {
    Serial.println("[boot] FATAL: BLE message queue allocation failed");
    return;
  }

  piBleTopic = ardb.addTopic("a/pi/ble", ARDBVisualType::Binary,
                             "Raspberry Pi BLE data");
  ardb.begin();

  BLEDevice::init("ardb-pi-bridge");
  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new PiAdvertisementCallbacks());
  scanner->setActiveScan(true);
  scanner->setInterval(160);
  scanner->setWindow(80);
  nextScanAt = millis();

  Serial.println("[boot] BLE-to-ARDB bridge ready");
}

void loop() {
  ardb.update();
  serviceBle();
  publishReceivedMessages();
  delay(2);
}
