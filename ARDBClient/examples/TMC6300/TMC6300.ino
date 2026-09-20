/**
 * TMC6300 PWM telemetry over BLE
 *
 * Board:  Raspberry Pi Pico 2 W (RP2350)
 * Driver: TMC6300 in 6-PWM mode
 * Library: SimpleFOC
 *
 * This sketch replaces the old direct Pico Wi-Fi/MQTT publisher. It drives
 * the motor exactly as before, but advertises a BLE GATT service and notifies
 * three high-side PWM duties as a 12-byte payload (three IEEE-754 floats)
 * every 100 ms. Flash the companion ESP32 sketch at:
 *
 *   Demo/pico_tmc6300_ble_ardb_bridge/esp32_pico_tmc6300_bridge.ino
 *
 * The ESP32 connects to this Pico, then publishes the unchanged payload on
 * MQTT topic b/demo/a through ARDBClient. This keeps the Pico independent of
 * the weak Wi-Fi link while preserving the original ARDB visual type and topic.
 *
 * Select "Raspberry Pi Pico 2 W" in Arduino IDE. This needs the RP2040
 * Arduino core's built-in BLE library (6.1.0 or newer).
 *
 * TMC6300 wiring:
 *   UH=GP0, UL=GP1, VH=GP2, VL=GP3, WH=GP4, WL=GP5
 *   VIO=3.3V, VM=motor supply (2..11V), GND shared with the Pico
 */

#include <Arduino.h>
#include <BLE.h>
#include <SimpleFOC.h>
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// These UUIDs differ from the generic Raspberry Pi bridge so the ESP32
// companion can only select this TMC6300/Pico telemetry service.
constexpr char kPwmServiceUuid[] = "a148b750-92c6-4f9a-9f61-a9f1d376adc1";
constexpr char kPwmCharacteristicUuid[] =
    "a148b751-92c6-4f9a-9f61-a9f1d376adc1";
constexpr size_t kPwmPayloadBytes = 3 * sizeof(float);
constexpr uint32_t kPwmReportIntervalMs = 100;

static_assert(sizeof(float) == 4,
              "The ESP32 bridge expects three 32-bit IEEE-754 floats.");

struct PwmState {
  bool pwmMuxed;
  bool enabled;
  uint16_t level;
  uint16_t top;
  float duty;
};

class PwmBleService final : public BLEService, public BLEServerCallbacks {
 public:
  PwmBleService() : BLEService(BLEUUID(kPwmServiceUuid)) {
    dataCharacteristic_ = new BLECharacteristic(
        BLEUUID(kPwmCharacteristicUuid), BLERead | BLENotify,
        "Three TMC6300 high-side PWM duties");
    const uint8_t initialValue[kPwmPayloadBytes] = {};
    dataCharacteristic_->setValue(initialValue, sizeof(initialValue));
    addCharacteristic(dataCharacteristic_);
  }

  void publish(const uint8_t* payload, size_t length) {
    if (connected_ && length == kPwmPayloadBytes) {
      // setValue() updates the GATT value and sends a notification after the
      // ESP32 has enabled the characteristic's notification descriptor.
      dataCharacteristic_->setValue(payload, length);
    }
  }

  bool takeAdvertisingRestartRequest() {
    if (!advertisingRestartRequested_) {
      return false;
    }
    advertisingRestartRequested_ = false;
    return true;
  }

 private:
  void onConnect(BLEServer*) override {
    connected_ = true;
    Serial.println("BLE bridge connected");
  }

  void onDisconnect(BLEServer*) override {
    connected_ = false;
    // Request the restart from loop(), not from the Bluetooth callback.
    advertisingRestartRequested_ = true;
    Serial.println("BLE bridge disconnected");
  }

  BLECharacteristic* dataCharacteristic_ = nullptr;
  volatile bool connected_ = false;
  volatile bool advertisingRestartRequested_ = false;
};

PwmBleService pwmBleService;

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
#define MOTOR_VOLT_LIMIT     3.f     // Start low; increase only if needed.
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

static void publishPwmTelemetry() {
  static uint32_t lastReportedAtMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastReportedAtMs < kPwmReportIntervalMs) {
    return;
  }
  lastReportedAtMs = nowMs;

  const float highSideDuties[3] = {
      readPwm(PIN_UH).duty,
      readPwm(PIN_VH).duty,
      readPwm(PIN_WH).duty,
  };
  uint8_t payload[kPwmPayloadBytes];
  memcpy(payload, highSideDuties, sizeof(payload));
  pwmBleService.publish(payload, sizeof(payload));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

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

  BLE.begin("ARDB-PWM");
  BLE.server()->addService(&pwmBleService);
  BLE.server()->setCallbacks(&pwmBleService);
  BLE.startAdvertising();
  Serial.println("TMC6300 BLE PWM peripheral ready");
}

void loop() {
  if (pwmBleService.takeAdvertisingRestartRequest()) {
    BLE.startAdvertising();
    Serial.println("BLE advertising restarted");
  }

  // velocity_openloop integrates this target into a rotating electrical angle.
  // A positive target drives one direction; a negative target drives the other.
  motor.target = sineVelocityTarget();
  motor.loopFOC();
  motor.move();

  publishPwmTelemetry();
}
