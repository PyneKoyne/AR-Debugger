/**
 * One-motor CW <-> CCW sine-wave test
 *
 * Board:  Raspberry Pi Pico 2 W (RP2350)
 * Driver: TMC6300 in 6-PWM mode
 * Library: SimpleFOC
 *
 * The motor is run open-loop, so no encoder is required. Its velocity is a
 * sine wave: it smoothly accelerates CW, slows to a stop, accelerates CCW,
 * and repeats. Change MAX_VELOCITY_RAD_S and SWEEP_PERIOD_MS below to tune
 * the motion. Start with a low MOTOR_VOLT_LIMIT and raise it cautiously.
 *
 * TMC6300 wiring:
 *   UH=GP0, UL=GP1, VH=GP2, VL=GP3, WH=GP4, WL=GP5
 *   VIO=3.3V, VM=motor supply (2..11V), GND shared with the Pico
 *
 * If CW and CCW are reversed for the mechanical setup, set
 * REVERSE_DIRECTION to true (or swap any two motor phase wires).
 */

#include <WiFi.h>
#include <Arduino.h>
#include <SimpleFOC.h>
#include <ARDBClient.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"

const char* WIFI_SSID = "Broker1";
const char* WIFI_PASSWORD = "abcdefghi";
const char* MQTT_HOST = "192.168.4.1";  // MQTT broker IP, not the Wi-Fi SSID
const uint16_t MQTT_PORT = 1883;

WiFiClient network;

// Keep this declaration above the first function in the sketch. Arduino's
// sketch preprocessor generates function prototypes before later declarations.
struct PwmState {
  bool pwmMuxed;
  bool enabled;
  uint16_t level;
  uint16_t top;
  float duty;
};

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
    WIFI_SSID, WIFI_PASSWORD, MQTT_HOST, "demo-pico-02", MQTT_PORT);
ARDBClient ardb(network, ardbNetwork, ardbConfig);
ARDBTopic pwmTopic;

PwmState readPwm(uint8_t pin) {
  const uint slice = pwm_gpio_to_slice_num(pin);
  const bool channelA = pwm_gpio_to_channel(pin) == PWM_CHAN_A;
  const uint32_t cc = pwm_hw->slice[slice].cc;
  const uint16_t top = pwm_hw->slice[slice].top;
  const uint16_t level = channelA
      ? static_cast<uint16_t>(cc & 0xffff)
      : static_cast<uint16_t>(cc >> 16);

  return {
    gpio_get_function(pin) == GPIO_FUNC_PWM,
    (pwm_hw->en & (1u << slice)) != 0,
    level,
    top,
    100.0f * level / (top + 1.0f)
  };
}

// ========================= USER CONFIG =========================
#define POLE_PAIRS           11      // Set this for the connected motor.
#define SUPPLY_VOLTAGE       6.0f    // TMC6300 VM voltage, in volts (2..11 V).
#define MOTOR_VOLT_LIMIT     3.f    // Start low; increase only if needed.
#define MAX_VELOCITY_RAD_S   15.0f   // Peak CW/CCW speed of the sine wave.
#define SWEEP_PERIOD_MS      8000UL  // Time for one full CW -> CCW -> CW cycle.
#define REVERSE_DIRECTION    false

// Each high/low phase pair shares a Pico PWM slice.
#define PIN_UH 0
#define PIN_UL 1
#define PIN_VH 2
#define PIN_VL 3
#define PIN_WH 4
#define PIN_WL 5
// ===============================================================

BLDCMotor motor(POLE_PAIRS);
BLDCDriver6PWM driver(PIN_UH, PIN_UL, PIN_VH, PIN_VL, PIN_WH, PIN_WL);

static float sineVelocityTarget() {
  const float phase = (TWO_PI * static_cast<float>(millis())) /
                      static_cast<float>(SWEEP_PERIOD_MS);
  const float direction = REVERSE_DIRECTION ? -1.0f : 1.0f;
  return direction * MAX_VELOCITY_RAD_S * sinf(phase);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // Start associating before the motor setup. The ARDB callbacks continue to
  // own retries after this initial request.
  Serial.println("Starting Wi-Fi association with Broker1");
  beginArdbNetwork(WIFI_SSID, WIFI_PASSWORD);

  driver.pwm_frequency = 32000;
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE;
  driver.dead_zone = 0.02f;
  if (!driver.init()) {
    while (true) {
      Serial.println(F("TMC6300 driver init failed; check the six PWM pins."));
      delay(1000);
    }
  }

  motor.linkDriver(&driver);
  motor.voltage_limit = MOTOR_VOLT_LIMIT;
  motor.velocity_limit = MAX_VELOCITY_RAD_S;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  Serial.println("HELLO");

  pwmTopic = ardb.addTopic("b/demo/a", ARDBVisualType::THREE_NUM,
                           "Demo BLDC PWM", 12);
  ardbConfig.mqttConnectTimeoutMs = 1000;
  ardb.begin();
}

void loop() {
  ardb.update();  // never loops until connected; call every iteration
  static bool connectionReported = false;
  if (ardb.connected() && !connectionReported) {
    ardb.print(pwmTopic, "ARDB connected");  // text length is inferred
    connectionReported = true;
  } else if (!ardb.connected()) {
    connectionReported = false;
  }

  // velocity_openloop integrates this target into a rotating electrical angle.
  // A positive target drives one direction; a negative target drives the other.
  motor.target = sineVelocityTarget();
  motor.loopFOC();
  motor.move();

  float uh = readPwm(0).duty;  // GP0
  float vh = readPwm(2).duty;  // GP2
  float wh = readPwm(4).duty;  // GP4

  // A 96-bit array (12 bytes)
  uint8_t bitArray96[12];

  // Copy each 4-byte integer into the array
  memcpy(bitArray96,     &uh, 4);
  memcpy(bitArray96 + 4, &vh, 4);
  memcpy(bitArray96 + 8, &wh, 4);

  ardb.print(pwmTopic, bitArray96, 12);
}
