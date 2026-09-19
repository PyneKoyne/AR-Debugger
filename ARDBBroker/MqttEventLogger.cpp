#include "MqttEventLogger.h"

MqttEventLogger* MqttEventLogger::instance_ = nullptr;

MqttEventLogger::MqttEventLogger(MqttBroker& broker, const char* loggerId)
    : broker_(broker), loggerClient_(&broker, loggerId)
{
}

void MqttEventLogger::begin()
{
  instance_ = this;
  loggerClient_.setCallback(onMqttPublish);
  loggerClient_.subscribe(Topic("#"));
}

void MqttEventLogger::update()
{
  logClientEvents();
}

void MqttEventLogger::onMqttPublish(const MqttClient*, const Topic& topic,
                                    const char* payload, size_t payloadLength)
{
  if (instance_ != nullptr) {
    instance_->logMqttPublish(topic, payload, payloadLength);
  }
}

void MqttEventLogger::logMqttPublish(const Topic& topic, const char* payload,
                                     size_t payloadLength)
{
  Serial.print("[MQTT] PUBLISH topic=\"");
  Serial.print(topic.c_str());
  Serial.print("\" payload=\"");
  printEscapedPayload(payload, payloadLength);
  Serial.println("\"");
}

void MqttEventLogger::printEscapedPayload(const char* payload, size_t length)
{
  const size_t maxLoggedBytes = 256;
  const size_t bytesToPrint = length < maxLoggedBytes ? length : maxLoggedBytes;

  for (size_t i = 0; i < bytesToPrint; ++i) {
    const uint8_t byte = static_cast<uint8_t>(payload[i]);
    if (byte == '\\' || byte == '\"') {
      Serial.write('\\');
      Serial.write(byte);
    } else if (byte >= 32 && byte <= 126) {
      Serial.write(byte);
    } else {
      char escaped[5];
      snprintf(escaped, sizeof(escaped), "\\x%02X", byte);
      Serial.print(escaped);
    }
  }

  if (length > maxLoggedBytes) {
    Serial.print("... (truncated)");
  }
}

void MqttEventLogger::logClientEvents()
{
  const std::vector<MqttClient*> clients = broker_.getClients();
  std::vector<SeenClient> currentClients;

  for (const MqttClient* client : clients) {
    if (client == &loggerClient_) {
      continue;
    }

    const String clientId = client->id().c_str();
    bool found = false;
    for (SeenClient& seen : seenClients_) {
      if (seen.client == client) {
        found = true;
        if (seen.id.length() == 0 && clientId.length() > 0) {
          Serial.print("[MQTT] CONNECT client=\"");
          Serial.print(clientId);
          Serial.println("\"");
        }
        seen.id = clientId;
        break;
      }
    }

    if (!found) {
      Serial.println("[MQTT] TCP client accepted");
      if (clientId.length() > 0) {
        Serial.print("[MQTT] CONNECT client=\"");
        Serial.print(clientId);
        Serial.println("\"");
      }
    }

    currentClients.push_back({client, clientId});
  }

  for (const SeenClient& seen : seenClients_) {
    bool stillConnected = false;
    for (const SeenClient& current : currentClients) {
      if (current.client == seen.client) {
        stillConnected = true;
        break;
      }
    }
    if (!stillConnected) {
      Serial.print("[MQTT] DISCONNECT");
      if (seen.id.length() > 0) {
        Serial.print(" client=\"");
        Serial.print(seen.id);
        Serial.print("\"");
      }
      Serial.println();
    }
  }

  seenClients_ = currentClients;
}
