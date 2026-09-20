// arduino_secrets.h -- fill in and keep out of version control.
//
// NOTE: .gitignore currently only excludes
// Demo/esp32cam_openai_vision/arduino_secrets.h, so this file is NOT ignored
// yet. Add "Demo/esp32_radar_servo/arduino_secrets.h" to .gitignore before
// committing real credentials.

#pragma once

// Wi-Fi (ESP32 is 2.4 GHz only)
#define SECRET_SSID       "YOUR_SSID"
#define SECRET_PASS       "YOUR_PASSWORD"

// ARDB broker (the head's IP, not the Wi-Fi SSID)
#define SECRET_MQTT_HOST  "10.37.114.246"
#define SECRET_MQTT_PORT  1883
