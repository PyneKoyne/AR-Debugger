/*
 * ESP32 servo radar -> ARDB (MQTT)
 *
 *   A servo pans an HC-SR04 ultrasonic sensor across a 180-degree arc. Every
 *   step publishes one polar sample (theta, r) to the ARDB head, plus the
 *   Cartesian projection of that sample and a per-sweep nearest-target report.
 *
 *   Servo signal  GPIO 32   (50 Hz LEDC, no servo library needed)
 *   Ultrasonic    GPIO 33 = TRIG (out), GPIO 25 = ECHO (in)
 *
 * !! ECHO IS 5 V ON A PLAIN HC-SR04 AND THE ESP32 IS NOT 5 V TOLERANT !!
 *   Use a 3.3 V part (HC-SR04P / RCWL-1601) or drop ECHO with a divider,
 *   e.g. 1 kOhm from ECHO to GPIO 25 and 2 kOhm from GPIO 25 to GND.
 *   Power the servo from its own 5 V supply with a common ground; it will
 *   brown out the ESP32 if run from the dev board's regulator.
 *
 * Requires an arduino_secrets.h tab defining:
 *   SECRET_SSID, SECRET_PASS, SECRET_MQTT_HOST, SECRET_MQTT_PORT
 *
 * Libraries: ArduinoMqttClient, ARDBClient
 * Board: any ESP32 dev module (tested layout: ESP32-WROOM-32)
 *
 * BISECTING: set ENABLE_ARDB to 0 to sweep and print to Serial with no network.
 */

#include <WiFi.h>
#include <math.h>

#include "arduino_secrets.h"

// ============================================================
//  FEATURE SWITCHES
// ============================================================
#define ENABLE_ARDB 1

#if ENABLE_ARDB
#include <ARDBClient.h>
#endif

// ============================================================
//  PINS
// ============================================================
static const int SERVO_PIN = 32;
static const int TRIG_PIN  = 33;
static const int ECHO_PIN  = 25;

// ============================================================
//  CONFIG  (secrets come from arduino_secrets.h)
// ============================================================
const char *WIFI_SSID = SECRET_SSID;
const char *WIFI_PASS = SECRET_PASS;

const char *MQTT_HOST = SECRET_MQTT_HOST;
const uint16_t MQTT_PORT = SECRET_MQTT_PORT;
const char *ARDB_CLIENT_ID = "esp32-radar-01";

// ------------------------------------------------------------
//  Sweep geometry
// ------------------------------------------------------------
// The servo's mechanical range is the radar's angular field of view. Keep
// SWEEP_MIN/MAX inside what the horn can actually reach without stalling.
static const float SWEEP_MIN_DEG  = 0.0f;
static const float SWEEP_MAX_DEG  = 180.0f;
static const float SWEEP_STEP_DEG = 6.0f;   // 30 steps per pass

// ------------------------------------------------------------
//  Sweep cadence  -- why 100 ms
// ------------------------------------------------------------
// Three limits meet here and 100 ms is the smallest period that satisfies all
// three, so every point we publish actually reaches the Quest:
//
//   1. HC-SR04 wants >= 60 ms between triggers or the previous burst's echo
//      ringing is still in the air when the next one goes out.
//   2. A 6 deg step takes an SG90-class servo ~15 ms plus settling; measuring
//      one full period after the move command means it is parked by then.
//   3. The ARDB head emits deltas no more often than every 100 ms
//      (kDeltaMinIntervalMs in ARDBBroker/src/HeadConfig.h). Publishing faster
//      than that does not produce more samples on the Quest, it just discards
//      the intermediate ones.
//
// At 6 deg and 100 ms a 0->180 pass takes 30 steps = 3.0 s.
static const uint32_t STEP_PERIOD_MS = 100;

// The servo needs longer to cross the whole arc than it does to take one step,
// so hold still after boot and after each direction flip's first command.
static const uint32_t SERVO_HOME_SETTLE_MS = 600;

// ------------------------------------------------------------
//  Ultrasonic ranging
// ------------------------------------------------------------
// 0.0343 cm/us at ~20 C, halved because the burst makes a round trip.
static const float CM_PER_US = 0.0343f / 2.0f;

static const float MIN_RANGE_CM = 3.0f;    // below this the module lies
static const float MAX_RANGE_CM = 200.0f;  // trim to the room you are testing in

// pulseIn() bounds the whole wait, so this is also the worst-case time the
// sketch spends unable to service ARDB. Derived from MAX_RANGE_CM + margin.
static const uint32_t ECHO_TIMEOUT_US =
    (uint32_t)((MAX_RANGE_CM / CM_PER_US) * 1.2f);

// ------------------------------------------------------------
//  Servo pulse widths
// ------------------------------------------------------------
// Datasheet values for a 9 g hobby servo. Trim these to the horn's real travel
// if 0/180 stalls the gears -- a stalled servo is the usual cause of a brownout
// reset mid-sweep.
static const uint32_t SERVO_MIN_US = 500;   // maps to SWEEP_MIN_DEG
static const uint32_t SERVO_MAX_US = 2500;  // maps to SWEEP_MAX_DEG
static const uint32_t SERVO_FREQ_HZ = 50;
static const uint8_t  SERVO_RES_BITS = 16;
#if ESP_ARDUINO_VERSION_MAJOR < 3
static const uint8_t  SERVO_LEDC_CHANNEL = 0;  // core 2.x addresses a channel
#endif

static const uint32_t WIFI_DOWN_RECONNECT_MS = 10000;

// ============================================================
//  ARDB WIRE SCHEMA
// ============================================================
// ARDB metadata carries a type byte and a byte count. It does NOT carry field
// order, units, or byte order, so those are fixed here and must be mirrored by
// the Quest decoder. See ARDBClient/Quest_Type_Report.md.
//
// Frame: the sensor sweeps the horizontal plane. Theta is measured counter-
// clockwise from the sensor's right-hand axis, matching the servo's own 0..180
// travel, so theta = 0 points right, 90 straight ahead, 180 left.
//
// All floats below are IEEE-754 binary32 LITTLE-ENDIAN (the ESP32's in-memory
// layout, copied verbatim by ARDBClient).
//
//   a/radar/polar  THREE_NUM   12 B  float[3] { thetaDeg, rCm, valid }
//                  valid = 1.0 for a real echo, 0.0 for no echo / out of
//                  range. rCm is 0.0 when valid is 0.0 -- read the flag, not
//                  the radius, or an empty bearing plots as a hit at the
//                  sensor's own origin.
//
//   a/radar/pt     Vector3F32  12 B  float[3] { x, y, z } in METRES
//                  Unity-handed: +x right, +y up, +z forward, origin at the
//                  sensor face. y is always 0 because the servo only pans.
//                  Published ONLY for valid echoes, so an absent sample means
//                  "nothing at that bearing", not "something at 0,0,0".
//
//   a/radar/r      ScalarF32    4 B  float rCm, valid echoes only.
//
//   a/radar/near   THREE_NUM   12 B  float[3] { thetaDeg, rCm, sweepIndex }
//                  The closest echo of a completed pass, emitted once per
//                  pass. sweepIndex counts passes from boot; it also keeps
//                  consecutive payloads byte-distinct, which matters because
//                  the head drops a sample identical to the one it already
//                  holds.
//
//   a/radar/log    Log               UTF-8 status text, no NUL terminator.

// ============================================================
//  ARDB - constructed in setup(), NOT as globals.
//  Building a WiFiClient before the TCP/IP stack is up faults before
//  Serial.begin() has run, which looks like a silent reset with no output.
// ============================================================
#if ENABLE_ARDB
static WiFiClient *ardbTransport = nullptr;
static ARDBConfig *ardbConfig = nullptr;
static ARDBNetworkCallbacks *ardbNetwork = nullptr;
static ARDBClient *ardb = nullptr;

static ARDBTopic tPolar;    // 1
static ARDBTopic tPoint;    // 2
static ARDBTopic tRange;    // 3
static ARDBTopic tNearest;  // 4
static ARDBTopic tLog;      // 5

// ARDBClient's per-topic gate restarts from the moment a publish *finishes*,
// so a 10 Hz limit against a 100 ms step drops a point whenever a publish
// takes more than 0 ms -- which is always. Give the client headroom and let
// the head's 100 ms delta interval be the real pacer.
static const uint16_t ARDB_PUBLISH_RATE_HZ = 25;
#endif

// ============================================================
//  SERVO  (LEDC directly; no servo library dependency)
// ============================================================
// The ESP32 Arduino core renamed the LEDC API in 3.x. Both spellings below
// produce the same 50 Hz / 16-bit channel.
static void servoBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, SERVO_FREQ_HZ, SERVO_RES_BITS);
#else
  ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
#endif
}

static void servoWriteMicroseconds(uint32_t pulseUs) {
  const uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;
  if (pulseUs > periodUs) pulseUs = periodUs;

  // duty = pulse / period, scaled to the timer's full 16-bit count.
  const uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1UL;
  const uint32_t duty = (uint32_t)(((uint64_t)pulseUs * maxDuty) / periodUs);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
#endif
}

static void servoWriteDegrees(float deg) {
  if (deg < SWEEP_MIN_DEG) deg = SWEEP_MIN_DEG;
  if (deg > SWEEP_MAX_DEG) deg = SWEEP_MAX_DEG;

  const float span = SWEEP_MAX_DEG - SWEEP_MIN_DEG;
  const float t = (span <= 0.0f) ? 0.0f : (deg - SWEEP_MIN_DEG) / span;
  servoWriteMicroseconds(
      SERVO_MIN_US + (uint32_t)(t * (float)(SERVO_MAX_US - SERVO_MIN_US)));
}

// ============================================================
//  ULTRASONIC
// ============================================================
static void sonarBegin() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

// Returns true and fills outCm on a usable echo. A false return is a real
// answer -- "nothing within MAX_RANGE_CM at this bearing" -- not an error.
static bool sonarPingCm(float &outCm) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return false;  // timed out waiting for the echo

  const float cm = (float)us * CM_PER_US;
  if (cm < MIN_RANGE_CM || cm > MAX_RANGE_CM) return false;

  outCm = cm;
  return true;
}

// ============================================================
//  WI-FI  (owned by the sketch; ARDB only gets a status callback)
// ============================================================
static bool wifiIsConnected() { return WiFi.status() == WL_CONNECTED; }

static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[wifi] disconnected, reason %u\n",
                    (unsigned)info.wifi_sta_disconnected.reason);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[wifi] got IP %s, RSSI %d\n",
                    WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
      break;
    default:
      break;
  }
}

static void wifiStart() {
  WiFi.onEvent(onWifiEvent);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // sleep adds tens of ms of jitter to every publish
  WiFi.setAutoReconnect(true);
  // Default FAST_SCAN joins the first matching AP, often a weak one on a
  // multi-AP network. Scan every channel and take the strongest.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// Re-joins instead of waiting when the link has been down a while.
static void netSupervisor() {
  static uint32_t nextCheck = 0;
  static uint32_t downSince = 0;

  const uint32_t now = millis();
  if ((int32_t)(now - nextCheck) < 0) return;
  nextCheck = now + 1000;

  if (wifiIsConnected()) {
    downSince = 0;
    return;
  }
  if (!downSince) downSince = now;

  if ((now - downSince) > WIFI_DOWN_RECONNECT_MS) {
    Serial.println("[net] re-joining Wi-Fi");
    downSince = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ============================================================
//  SWEEP STATE
// ============================================================
static float    g_angleDeg = SWEEP_MIN_DEG;  // bearing the horn is parked at
static float    g_stepDeg = SWEEP_STEP_DEG;  // signed: flips at each end
static uint32_t g_nextStepAt = 0;
static uint32_t g_sweepIndex = 0;

// Nearest echo seen so far in the pass currently underway.
static bool  g_sweepHasHit = false;
static float g_sweepMinCm = 0.0f;
static float g_sweepMinDeg = 0.0f;

static void publishSample(float thetaDeg, bool valid, float rCm) {
#if ENABLE_ARDB
  const float polar[3] = {thetaDeg, valid ? rCm : 0.0f, valid ? 1.0f : 0.0f};
  ardb->print(tPolar, polar);

  if (valid) {
    // Polar -> Cartesian, cm -> m. y stays 0: the servo only pans.
    const float rM = rCm / 100.0f;
    const float rad = thetaDeg * (float)M_PI / 180.0f;
    const float point[3] = {rM * cosf(rad), 0.0f, rM * sinf(rad)};
    ardb->print(tPoint, point);
    ardb->print(tRange, rCm);
  }
#else
  (void)thetaDeg;
  (void)valid;
  (void)rCm;
#endif
}

static void publishSweepSummary() {
  if (!g_sweepHasHit) {
    Serial.printf("[sweep %lu] no echo\n", (unsigned long)g_sweepIndex);
    return;  // an empty pass has no nearest target to report
  }
#if ENABLE_ARDB
  const float nearest[3] = {g_sweepMinDeg, g_sweepMinCm, (float)g_sweepIndex};
  ardb->print(tNearest, nearest);
#endif
  Serial.printf("[sweep %lu] nearest %.1f cm at %.0f deg\n",
                (unsigned long)g_sweepIndex, g_sweepMinCm, g_sweepMinDeg);
}

// Folds one sample into the running nearest-target search.
static void trackNearest(float thetaDeg, float rCm) {
  if (!g_sweepHasHit || rCm < g_sweepMinCm) {
    g_sweepHasHit = true;
    g_sweepMinCm = rCm;
    g_sweepMinDeg = thetaDeg;
  }
}

// Advances g_angleDeg one step, reversing and closing out the pass at an end.
static void advanceSweep() {
  float next = g_angleDeg + g_stepDeg;

  if (next > SWEEP_MAX_DEG || next < SWEEP_MIN_DEG) {
    publishSweepSummary();
    ++g_sweepIndex;
    g_sweepHasHit = false;

    g_stepDeg = -g_stepDeg;
    next = g_angleDeg + g_stepDeg;
    if (next > SWEEP_MAX_DEG) next = SWEEP_MAX_DEG;
    if (next < SWEEP_MIN_DEG) next = SWEEP_MIN_DEG;
  }

  g_angleDeg = next;
  servoWriteDegrees(g_angleDeg);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 servo radar -> ARDB ===");
  Serial.printf("[boot] servo=GPIO%d trig=GPIO%d echo=GPIO%d\n", SERVO_PIN,
                TRIG_PIN, ECHO_PIN);
  Serial.printf("[boot] sweep %.0f..%.0f deg, step %.1f deg, %lu ms/step\n",
                SWEEP_MIN_DEG, SWEEP_MAX_DEG, SWEEP_STEP_DEG,
                (unsigned long)STEP_PERIOD_MS);

  sonarBegin();
  servoBegin();

  // Park at the start of the arc and let the horn get there before the first
  // ping, otherwise sample 1 is taken somewhere between the two bearings.
  g_angleDeg = SWEEP_MIN_DEG;
  g_stepDeg = SWEEP_STEP_DEG;
  servoWriteDegrees(g_angleDeg);

  wifiStart();

#if ENABLE_ARDB
  Serial.println("[boot] constructing ARDB");
  ardbTransport = new WiFiClient();
  ardbConfig = new ARDBConfig(ARDBConfig::wifiMqtt(
      WIFI_SSID, WIFI_PASS, MQTT_HOST, ARDB_CLIENT_ID, MQTT_PORT));
  // beginNetwork is null: the sketch already owns Wi-Fi, ARDB only polls it.
  ardbNetwork = new ARDBNetworkCallbacks(nullptr, wifiIsConnected);
  ardb = new ARDBClient(*ardbTransport, *ardbNetwork, *ardbConfig,
                        ARDB_PUBLISH_RATE_HZ);

  // Registration order fixes the metadata topic suffix (ardb/meta/<id>/1..5).
  // The head assigns its own stream IDs; the Quest must use those, not these.
  tPolar   = ardb->addTopic("a/radar/polar", ARDBVisualType::THREE_NUM,
                            "Radar polar deg/cm/valid", 12);
  tPoint   = ardb->addTopic("a/radar/pt", ARDBVisualType::Vector3F32,
                            "Radar point XYZ m", 12);
  tRange   = ardb->addTopic("a/radar/r", ARDBVisualType::ScalarF32,
                            "Radar range cm", 4);
  tNearest = ardb->addTopic("a/radar/near", ARDBVisualType::THREE_NUM,
                            "Nearest deg/cm/sweep", 12);
  tLog     = ardb->addTopic("a/radar/log", ARDBVisualType::Log, "Radar status");

  if (!tPolar.valid() || !tPoint.valid() || !tRange.valid() ||
      !tNearest.valid() || !tLog.valid()) {
    Serial.println("[boot] WARNING: a topic failed to register");
  }

  Serial.println("[boot] ardb.begin()");
  ardb->begin();
#endif

  delay(SERVO_HOME_SETTLE_MS);
  g_nextStepAt = millis();
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
#if ENABLE_ARDB
  ardb->update();  // drives Wi-Fi/MQTT state; call every iteration

  static bool connectionReported = false;
  if (ardb->connected() && !connectionReported) {
    connectionReported = true;
    ardb->print(tLog, "esp32 radar online");
    Serial.println("[ardb] connected");
  } else if (!ardb->connected() && connectionReported) {
    connectionReported = false;
    Serial.println("[ardb] disconnected");
  }
#endif

  netSupervisor();

  const uint32_t now = millis();
  if ((int32_t)(now - g_nextStepAt) < 0) return;

  // Fixed cadence rather than now + PERIOD, so a slow publish does not stretch
  // the sweep. Resynchronise if we ever fall a whole step behind.
  g_nextStepAt += STEP_PERIOD_MS;
  if ((int32_t)(now - g_nextStepAt) > 0) g_nextStepAt = now + STEP_PERIOD_MS;

  // The horn was commanded to g_angleDeg one full period ago, so it is parked.
  float rCm = 0.0f;
  const bool valid = sonarPingCm(rCm);

  if (valid) trackNearest(g_angleDeg, rCm);
  publishSample(g_angleDeg, valid, rCm);

  if (valid) {
    Serial.printf("%6.1f deg  %7.1f cm\n", g_angleDeg, rCm);
  } else {
    Serial.printf("%6.1f deg        --\n", g_angleDeg);
  }

  advanceSweep();
}
