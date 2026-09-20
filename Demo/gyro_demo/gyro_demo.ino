/*
 * gyro_demo - MPU-6050 attitude streaming to ARDB
 *
 *   loop():        read the IMU at 100 Hz, run a 2-state Kalman filter per
 *                  axis, publish attitude + raw counts + die temperature
 *   netSupervisor: repairs Wi-Fi instead of waiting on it, reboots as a last
 *                  resort
 *
 * Board:   NodeMCU / Wemos D1 (ESP8266). Also builds for ESP32.
 * Wiring:  MPU-6050 VCC->3V3, GND->GND, SDA->D2 (GPIO4), SCL->D1 (GPIO5).
 *          AD0 tied low or floating -> I2C address 0x68.
 *
 * Requires an arduino_secrets.h tab defining:
 *   SECRET_SSID, SECRET_PASS, SECRET_MQTT_HOST, SECRET_MQTT_PORT
 * Copy arduino_secrets.example.h to arduino_secrets.h to create it.
 *
 * Libraries: ArduinoMqttClient, ARDBClient
 *
 * BISECTING: set ENABLE_ARDB or ENABLE_IMU to 0 to cut out that subsystem.
 *
 * IMPORTANT: hold the board still for the first ~1.5 s after reset. That
 * window measures and removes the gyro's zero-rate bias.
 *
 * ---------------------------------------------------------------------------
 * ARDB stream contract (little-endian; ESP8266/ESP32 native byte order)
 * ---------------------------------------------------------------------------
 * The ARDB metadata type byte selects a Quest decoder, but field order, units
 * and scale are a publisher/Quest agreement that metadata does NOT carry.
 * This sketch's agreement is:
 *
 *  g/imu/r  Imu6I16T32 (16 B) raw sensor counts, packed, in this order:
 *             uint32 timeMs, int16 ax, ay, az, int16 gx, gy, gz
 *           Scale: accel 16384 counts/g (+/-2 g), gyro 131 counts/deg/s
 *           (+/-250 deg/s). Gyro counts are raw, i.e. bias NOT removed.
 *
 *  g/imu/k  Vector3F32 (12 B) filtered attitude, three float32 in order:
 *             [0] roll  deg   (rotation about X, +right-side-down)
 *             [1] pitch deg   (rotation about Y, +nose-up)
 *             [2] yawRate deg/s (Z gyro, bias-removed; yaw angle is not
 *                 observable with a 6-axis IMU, so a rate is sent instead)
 *
 *  g/imu/t  ScalarF32 (4 B) die temperature, float32 degrees Celsius.
 *
 *  g/imu/l  Log (variable) UTF-8 status text, no trailing NUL. Keep each
 *           message <= 64 bytes: the head drops longer application payloads.
 * ---------------------------------------------------------------------------
 */

#include <Wire.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include "arduino_secrets.h"

// ============================================================
//  FEATURE SWITCHES
// ============================================================
#define ENABLE_ARDB 1
#define ENABLE_IMU  1

#if ENABLE_ARDB
#include <ARDBClient.h>
#endif

// ============================================================
//  CONFIG  (secrets come from arduino_secrets.h)
// ============================================================
const char* WIFI_SSID = SECRET_SSID;
const char* WIFI_PASS = SECRET_PASS;

const char* MQTT_HOST = SECRET_MQTT_HOST;
const uint16_t MQTT_PORT = SECRET_MQTT_PORT;
// Must be unique per device: clean session is on, so a duplicate id makes the
// broker bounce both clients. It is also the metadata topic component.
const char* ARDB_CLIENT_ID = "demo-gyro-01";

#if defined(ARDUINO_ARCH_ESP8266)
#define I2C_SDA D2  // GPIO4
#define I2C_SCL D1  // GPIO5
#else
#define I2C_SDA 21
#define I2C_SCL 22
#endif

const uint8_t MPU_ADDR = 0x68;

// MPU-6050 register map subset
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_WHO_AM_I = 0x75;

// Full-scale conversions for the ranges configured in setup().
const float ACCEL_COUNTS_PER_G = 16384.0f;  // +/-2 g
const float GYRO_COUNTS_PER_DPS = 131.0f;   // +/-250 deg/s

const uint32_t SAMPLE_PERIOD_MS = 10;       // 100 Hz filter update
const uint16_t GYRO_CALIB_SAMPLES = 600;    // ~1.2 s of averaging
const uint32_t SERIAL_PLOT_PERIOD_MS = 50;  // Serial Plotter rate
const uint32_t HEARTBEAT_PERIOD_MS = 5000;

// Self-healing (see netSupervisor). A successful ARDB publish is this
// sketch's proof of end-to-end liveness, the way a completed HTTP response
// is in esp32cam_openai_vision.
const uint32_t WIFI_DOWN_RECONNECT_MS = 30000;  // link down this long -> re-join
const uint32_t FORCE_RECONNECT_GAP_MS = 30000;  // min gap between forced re-joins
const uint32_t PUBLISH_STALE_MS = 30000;        // link up but nothing sent -> re-join
const uint32_t REBOOT_AFTER_MS = 180000;        // nothing sent at all -> reboot

// ============================================================
//  WIRE STRUCTURES
//  Keep these above the first function: Arduino's sketch preprocessor
//  generates function prototypes before later declarations.
// ============================================================

// Packed and size-checked: a default C++ layout is never a wire schema.
struct __attribute__((packed)) ImuRawFrame {
  uint32_t timeMs;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};
static_assert(sizeof(ImuRawFrame) == 16, "Imu6I16T32 frame must be 16 bytes");

// ============================================================
//  KALMAN FILTER (2-state: angle + gyro bias)
// ============================================================
class KalmanFilter {
 public:
  KalmanFilter()
      : Q_angle(0.001f),
        Q_bias(0.003f),
        R_measure(0.03f),
        angle(0.0f),
        bias(0.0f) {
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    // Predict
    const float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    const float S = P[0][0] + R_measure;
    const float K0 = P[0][0] / S;
    const float K1 = P[1][0] / S;

    const float y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;

    const float P00 = P[0][0];
    const float P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;

    return angle;
  }

  void setAngle(float newAngle) { angle = newAngle; }
  float getBias() const { return bias; }

 private:
  float Q_angle, Q_bias, R_measure;
  float angle, bias;
  float P[2][2];
};

// ============================================================
//  ARDB - constructed in setup(), NOT as globals.
//  Global constructors run before Serial.begin() (and before FreeRTOS is
//  fully up on ESP32), so a fault there gives a silent reset with no output.
// ============================================================
#if ENABLE_ARDB
static WiFiClient* ardbTransport = nullptr;
static ARDBConfig* ardbConfig = nullptr;
static ARDBNetworkCallbacks* ardbNetwork = nullptr;
static ARDBClient* ardb = nullptr;

static bool wifiIsConnected() { return WiFi.status() == WL_CONNECTED; }

static ARDBTopic tRaw;          // 1
static ARDBTopic tAttitude;     // 2
static ARDBTopic tTemperature;  // 3
static ARDBTopic tLog;          // 4
static bool ardbReady = false;
#endif

// ============================================================
//  SHARED STATE
// ============================================================
static KalmanFilter kalmanRoll;
static KalmanFilter kalmanPitch;

static uint32_t lastSampleMicros = 0;
static bool imuOk = false;

static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;

// Network health, read by netSupervisor().
static uint32_t g_lastPublishOk = 0;  // last successful ARDB publish

// Set from the Wi-Fi event callbacks and drained in loop(). On the ESP8266
// these callbacks run in SYS context, where Serial output can fault, so the
// printing is deferred rather than done inline.
static volatile bool g_wifiGotIpEvent = false;
static volatile bool g_wifiLostEvent = false;
static volatile int g_wifiLostReason = 0;

#if defined(ARDUINO_ARCH_ESP8266)
static WiFiEventHandler wifiGotIpHandler;
static WiFiEventHandler wifiLostHandler;
#endif

// ============================================================
//  I2C HELPERS
//  `Wire.read() << 8 | Wire.read()` is a latent bug: the evaluation order of
//  the two calls is unspecified in C++, so the bytes can silently swap when
//  the compiler or toolchain changes. Read into named locals instead.
// ============================================================
bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, length, (uint8_t) true) != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

inline int16_t be16(const uint8_t* p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

// One burst read of ACCEL, TEMP and GYRO so all seven values share a sample
// instant.
bool readMpuBurst(int16_t* ax, int16_t* ay, int16_t* az, int16_t* temp,
                  int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t raw[14];
  if (!mpuRead(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
    return false;
  }
  *ax = be16(&raw[0]);
  *ay = be16(&raw[2]);
  *az = be16(&raw[4]);
  *temp = be16(&raw[6]);
  *gx = be16(&raw[8]);
  *gy = be16(&raw[10]);
  *gz = be16(&raw[12]);
  return true;
}

bool initMpu() {
  uint8_t who = 0;
  if (!mpuRead(REG_WHO_AM_I, &who, 1)) {
    Serial.println(F("[imu] no ACK at 0x68. Check SDA/SCL and 3V3."));
    return false;
  }
  Serial.printf("[imu] WHO_AM_I = 0x%02X\n", who);

  // Wake up: clear SLEEP and select the X gyro PLL as the clock source.
  if (!mpuWrite(REG_PWR_MGMT_1, 0x01)) {
    Serial.println(F("[imu] failed to clear SLEEP"));
    return false;
  }
  delay(50);

  mpuWrite(REG_SMPLRT_DIV, 0x07);    // 1 kHz / (1+7) = 125 Hz internal rate
  mpuWrite(REG_CONFIG, 0x03);        // DLPF 44 Hz accel / 42 Hz gyro
  mpuWrite(REG_GYRO_CONFIG, 0x00);   // +/-250 deg/s  -> 131 LSB/deg/s
  mpuWrite(REG_ACCEL_CONFIG, 0x00);  // +/-2 g        -> 16384 LSB/g
  delay(50);
  return true;
}

// ============================================================
//  ATTITUDE
// ============================================================
void accelAngles(int16_t ax, int16_t ay, int16_t az, float* roll, float* pitch) {
  // atan2 of raw counts is fine: only the ratio matters, so the scale factor
  // cancels. The cast to float keeps the sqrt in single precision.
  const float fax = (float)ax;
  const float fay = (float)ay;
  const float faz = (float)az;
  *roll = atan2f(fay, faz) * 180.0f / (float)PI;
  *pitch = atan2f(-fax, sqrtf(fay * fay + faz * faz)) * 180.0f / (float)PI;
}

void calibrateGyro() {
  Serial.println(F("[imu] calibrating gyro bias - hold still"));
  double sx = 0, sy = 0, sz = 0;
  uint16_t taken = 0;

  for (uint16_t i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    int16_t ax, ay, az, temp, gx, gy, gz;
    if (readMpuBurst(&ax, &ay, &az, &temp, &gx, &gy, &gz)) {
      sx += gx;
      sy += gy;
      sz += gz;
      taken++;
    }
    delay(2);  // also yields to the ESP8266 Wi-Fi stack
  }

  if (taken == 0) {
    Serial.println(F("[imu] calibration read failed, assuming zero bias"));
    return;
  }

  gyroBiasX = (float)(sx / taken) / GYRO_COUNTS_PER_DPS;
  gyroBiasY = (float)(sy / taken) / GYRO_COUNTS_PER_DPS;
  gyroBiasZ = (float)(sz / taken) / GYRO_COUNTS_PER_DPS;
  Serial.printf("[imu] gyro bias deg/s: x=%.3f y=%.3f z=%.3f (%u samples)\n",
                gyroBiasX, gyroBiasY, gyroBiasZ, taken);
}

// ============================================================
//  WI-FI  (owned by the sketch; ARDB only gets a status callback)
// ============================================================
// ESP8266 disconnect reasons worth knowing:
//   1 UNSPECIFIED   2 AUTH_EXPIRE   4 ASSOC_EXPIRE   15 4WAY_HANDSHAKE_TIMEOUT
//   201 NO_AP_FOUND   202 AUTH_FAIL   203 ASSOC_FAIL
static void wifiJoin() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void wifiStart() {
#if defined(ARDUINO_ARCH_ESP8266)
  wifiLostHandler = WiFi.onStationModeDisconnected(
      [](const WiFiEventStationModeDisconnected& event) {
        g_wifiLostReason = (int)event.reason;
        g_wifiLostEvent = true;
      });
  wifiGotIpHandler =
      WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event) {
        (void)event;
        g_wifiGotIpEvent = true;
      });
#else
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      g_wifiLostReason = (int)info.wifi_sta_disconnected.reason;
      g_wifiLostEvent = true;
    } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
      g_wifiGotIpEvent = true;
    }
  });
#endif

  WiFi.persistent(false);  // do not rewrite flash on every join
  WiFi.mode(WIFI_STA);
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
#else
  WiFi.setSleep(false);
  // Default is FAST_SCAN: join the FIRST matching AP. On a multi-AP network
  // that is often a weak one. Scan every channel and take the strongest.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
#endif
  WiFi.setAutoReconnect(true);
  wifiJoin();
}

// Drains the Wi-Fi event flags where Serial is safe to use.
static void reportWifiEvents() {
  if (g_wifiLostEvent) {
    g_wifiLostEvent = false;
    Serial.printf("[wifi] disconnected, reason %d\n", g_wifiLostReason);
  }
  if (g_wifiGotIpEvent) {
    g_wifiGotIpEvent = false;
    Serial.printf("[wifi] got IP %s, RSSI %d\n",
                  WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  }
}

// Runs once a second from loop(). Repairs the connection instead of waiting
// on it, and reboots as a last resort.
static void netSupervisor() {
  static uint32_t nextCheck = 0;
  static uint32_t downSince = 0;
  static uint32_t lastForce = 0;

  const uint32_t now = millis();
  if ((int32_t)(now - nextCheck) < 0) return;
  nextCheck = now + 1000;

  const bool up = WiFi.status() == WL_CONNECTED;
  if (up) {
    downSince = 0;
  } else if (!downSince) {
    downSince = now;
  }

  const bool stuckDown = !up && (now - downSince) > WIFI_DOWN_RECONNECT_MS;
#if ENABLE_ARDB
  // Link is up but nothing is reaching the broker: the association is
  // probably stale even though the driver still reports WL_CONNECTED.
  const bool publishDead = up && (now - g_lastPublishOk) > PUBLISH_STALE_MS;
#else
  const bool publishDead = false;
#endif

  if ((stuckDown || publishDead) && (now - lastForce) > FORCE_RECONNECT_GAP_MS) {
    lastForce = now;
    Serial.printf("[net] re-joining Wi-Fi (%s)\n",
                  stuckDown ? "link down too long" : "broker unreachable");
    WiFi.disconnect(false);
    wifiJoin();
  }

#if ENABLE_ARDB
  if ((now - g_lastPublishOk) > REBOOT_AFTER_MS) {
    Serial.println(F("[net] no successful publish for 3 min, rebooting"));
    Serial.flush();
    delay(100);
    ESP.restart();
  }
#endif
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("=== gyro_demo -> ARDB ==="));
  Serial.printf("[boot] heap %u\n", (unsigned)ESP.getFreeHeap());

#if ENABLE_IMU
  Serial.println(F("[boot] init IMU"));
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  imuOk = initMpu();
  if (!imuOk) {
    Serial.println(F("[boot] IMU unavailable. Check wiring and the 3V3 rail. "
                     "Continuing without it."));
  } else {
    calibrateGyro();

    // Seed both filters from the accelerometer so they do not spend the first
    // seconds converging from zero.
    int16_t ax, ay, az, temp, gx, gy, gz;
    if (readMpuBurst(&ax, &ay, &az, &temp, &gx, &gy, &gz)) {
      float roll, pitch;
      accelAngles(ax, ay, az, &roll, &pitch);
      kalmanRoll.setAngle(roll);
      kalmanPitch.setAngle(pitch);
      Serial.printf("[imu] seeded roll=%.2f pitch=%.2f\n", roll, pitch);
    }
  }
#endif

  // Wi-Fi MUST be started before anything touches a socket. The sketch owns
  // Wi-Fi; ARDB only gets a status callback.
  Serial.println(F("[boot] starting Wi-Fi"));
  wifiStart();

#if ENABLE_ARDB
  Serial.println(F("[boot] constructing ARDB"));
  ardbTransport = new WiFiClient();
  ardbConfig = new ARDBConfig(ARDBConfig::wifiMqtt(
      WIFI_SSID, WIFI_PASS, MQTT_HOST, ARDB_CLIENT_ID, MQTT_PORT,
      /* retrySeconds */ 3, /* enabled */ true));
  // ARDBClient copies the config, so every field must be final BEFORE the
  // constructor runs. connect() blocks waiting for CONNACK, and the 250 ms
  // default is often too tight for an ESP8266 also servicing its Wi-Fi stack.
  ardbConfig->mqttConnectTimeoutMs = 800;
  // beginNetwork is null: the sketch owns association, ARDB just waits for
  // the link and then owns MQTT retries.
  ardbNetwork = new ARDBNetworkCallbacks(nullptr, wifiIsConnected);
  // 10 Hz per topic matches the head's 100 ms delta floor.
  ardb = new ARDBClient(*ardbTransport, *ardbNetwork, *ardbConfig,
                        /* dataPublishRateHz */ 10);

  Serial.println(F("[boot] registering topics"));
  // Registration order fixes the metadata topic ids ardb/meta/<clientId>/1..4.
  // Keep it stable, and clear retained metadata if you rename a topic.
  tRaw = ardb->addTopic("g/imu/r", ARDBVisualType::Imu6I16T32,
                        "Gyro raw + timestamp", sizeof(ImuRawFrame));
  tAttitude = ardb->addTopic("g/imu/k", ARDBVisualType::Vector3F32,
                             "Roll / Pitch / YawRate deg", 12);
  tTemperature = ardb->addTopic("g/imu/t", ARDBVisualType::ScalarF32,
                                "IMU temperature C", 4);
  tLog = ardb->addTopic("g/imu/l", ARDBVisualType::Log, "Gyro demo log");

  if (!tRaw.valid() || !tAttitude.valid() || !tTemperature.valid() ||
      !tLog.valid()) {
    Serial.println(F("[ardb] FATAL: topic registration rejected"));
  }

  Serial.println(F("[boot] ardb->begin()"));
  ardb->begin();
  ardbReady = true;
#endif

  lastSampleMicros = micros();
  g_lastPublishOk = millis();  // start the connectivity clock at boot
  Serial.printf("[boot] setup complete, heap %u\n", (unsigned)ESP.getFreeHeap());
}

// ============================================================
//  LOOP - must never block
// ============================================================
void loop() {
  static uint32_t nextSampleMs = 0;
  static uint32_t nextPlotMs = 0;
  static uint32_t nextHeartbeat = 0;
  static bool wasConnected = false;
  static float lastRoll = 0.0f;
  static float lastPitch = 0.0f;
  static float lastYawRate = 0.0f;

#if ENABLE_ARDB
  if (ardbReady && ardb) {
    ardb->update();  // drives Wi-Fi status, MQTT and metadata

    const bool nowConnected = ardb->connected();
    if (nowConnected && !wasConnected) {
      Serial.println(F("[ardb] connected"));
      // Text goes to the variable-length Log topic. A fixed-size topic would
      // silently reject it on the length check.
      ardb->print(tLog, imuOk ? "gyro online" : "gyro online (no IMU)");
    } else if (!nowConnected && wasConnected) {
      Serial.println(F("[ardb] disconnected"));
    }
    wasConnected = nowConnected;
  }
#endif

  reportWifiEvents();
  netSupervisor();

  const uint32_t now = millis();

  if ((int32_t)(now - nextHeartbeat) >= 0) {
    nextHeartbeat = now + HEARTBEAT_PERIOD_MS;
    Serial.printf("[hb] up %lus heap %u wifi %d rssi %d", 
                  (unsigned long)(now / 1000), (unsigned)ESP.getFreeHeap(),
                  (int)WiFi.status(), (int)WiFi.RSSI());
#if ENABLE_ARDB
    if (ardb) {
      Serial.printf(" ardb %d (state %d, err %d, sent %lu, dropped %lu)",
                    ardb->connected() ? 1 : 0, (int)ardb->state(),
                    ardb->lastConnectError(),
                    (unsigned long)ardb->sentPackets(),
                    (unsigned long)ardb->droppedPackets());
    }
#endif
    Serial.println();
  }

  if ((int32_t)(now - nextSampleMs) >= 0) {
    nextSampleMs = now + SAMPLE_PERIOD_MS;

    int16_t ax, ay, az, rawTemp, gx, gy, gz;
    if (imuOk && readMpuBurst(&ax, &ay, &az, &rawTemp, &gx, &gy, &gz)) {
      // Unsigned subtraction so a micros() rollover still yields the true
      // elapsed interval.
      const uint32_t nowMicros = micros();
      float dt = (float)(nowMicros - lastSampleMicros) / 1000000.0f;
      lastSampleMicros = nowMicros;
      // Guard the filter against a stalled or absurd interval.
      if (dt <= 0.0f || dt > 0.2f) {
        dt = (float)SAMPLE_PERIOD_MS / 1000.0f;
      }

      float rollAcc, pitchAcc;
      accelAngles(ax, ay, az, &rollAcc, &pitchAcc);

      const float rollRate = (float)gx / GYRO_COUNTS_PER_DPS - gyroBiasX;
      const float pitchRate = (float)gy / GYRO_COUNTS_PER_DPS - gyroBiasY;
      lastYawRate = (float)gz / GYRO_COUNTS_PER_DPS - gyroBiasZ;

      lastRoll = kalmanRoll.getAngle(rollAcc, rollRate, dt);
      lastPitch = kalmanPitch.getAngle(pitchAcc, pitchRate, dt);

      // Datasheet transfer function for the on-die sensor.
      const float temperatureC = (float)rawTemp / 340.0f + 36.53f;

#if ENABLE_ARDB
      if (ardbReady && ardb && ardb->connected()) {
        const ImuRawFrame frame = {now, ax, ay, az, gx, gy, gz};
        // The typed overload infers sizeof(frame) == 16.
        bool anySent = ardb->print(tRaw, frame);

        const float attitude[3] = {lastRoll, lastPitch, lastYawRate};
        anySent |= ardb->print(tAttitude, attitude);  // 3 * 4 = 12 bytes

        // Must be float, not double: a double would be 8 bytes and fail the
        // 4-byte length check.
        anySent |= ardb->print(tTemperature, temperatureC);

        // Feeds netSupervisor(). Rate-limited calls return false, so this
        // only advances when bytes really reached the broker.
        if (anySent) g_lastPublishOk = millis();
      }
#endif
    }
  }

  if ((int32_t)(now - nextPlotMs) >= 0) {
    nextPlotMs = now + SERIAL_PLOT_PERIOD_MS;
    // Arduino Serial Plotter format, kept for local debugging.
    Serial.print(F("Roll:"));
    Serial.print(lastRoll, 2);
    Serial.print(F(" Pitch:"));
    Serial.print(lastPitch, 2);
    Serial.print(F(" YawRate:"));
    Serial.println(lastYawRate, 2);
  }

  delay(1);  // yields to the ESP8266 Wi-Fi stack
}
