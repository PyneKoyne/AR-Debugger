/*
 * Template for gyro_demo. Copy this file to `arduino_secrets.h` in the same
 * folder and fill in your own values. `arduino_secrets.h` is gitignored so
 * real credentials are never committed.
 *
 * SECRET_MQTT_HOST is the ARDB broker's IP address, NOT the Wi-Fi SSID.
 * 192.168.4.1 is the ARDBBroker SoftAP default.
 */
#ifndef ARDUINO_SECRETS_H
#define ARDUINO_SECRETS_H

#define SECRET_SSID      "Broker1"
#define SECRET_PASS      "abcdefghi"
#define SECRET_MQTT_HOST "192.168.4.1"
#define SECRET_MQTT_PORT 1883

#endif  // ARDUINO_SECRETS_H
