/*
 * ESP32 servo radar -> ARDB (MQTT)
 *
 *   A servo pans an HC-SR04 ultrasonic sensor across a 180-degree arc. Every
 *   step publishes ONE 8-byte packet to the ARDB head on ONE stream:
 *   float[2] { bearing in degrees, range in centimetres }.
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
static const int ECHO_PIN  = 13;

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
// ONE stream, ONE packet per sweep step, two fields and nothing else:
//
//   a/radar   Binary (255)   8 B   float[2] { thetaDeg, rCm }
//
//     [0] thetaDeg  bearing, 0..180, the servo angle described above.
//     [1] rCm       range in centimetres from the sensor face.
//
//     rCm == 0.0 is the NO-ECHO sentinel: the beam swept that bearing and
//     nothing answered. It is not a target at the sensor's origin. Zero is
//     safe as a sentinel because a real reading is always >= MIN_RANGE_CM,
//     and with only two fields there is nowhere else to put a validity flag.
//     A decoder that plots rCm without testing for zero will draw a false
//     contact at the origin on every empty bearing.
//
// Binary is the honest descriptor here: the enum has no two-number type, and
// Binary is defined as opaque application-defined bytes, so the length and
// this comment are the whole contract. A fixed 8 gives the head a length
// check. Nothing else is published -- no Cartesian point, no nearest-target
// summary, no log stream.

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

static ARDBTopic tRadar;   // the only stream

// float[2] {thetaDeg, rCm}. Asserted rather than assumed: the head enforces
// the 8 declared below, and a float that is not 4 bytes would be rejected at
// run time instead of here.
static const uint16_t RADAR_PACKET_BYTES = 8;
static_assert(sizeof(float) == 4, "ARDB radar packet assumes 32-bit float");
static_assert(sizeof(float[2]) == RADAR_PACKET_BYTES,
              "radar packet must be exactly 8 bytes");

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
static uint32_t g_sweepIndex = 0;   // completed passes, for the Serial log only

// One publish per sweep step: bearing and range, nothing else. An empty
// bearing still publishes, with rCm = 0 as the no-echo sentinel, so the
// viewer keeps getting a packet per step and can clear a stale contact
// instead of leaving the last hit on screen forever.
static void publishSample(float thetaDeg, bool valid, float rCm) {
#if ENABLE_ARDB
  const float packet[2] = {thetaDeg, valid ? rCm : 0.0f};
  ardb->print(tRadar, packet);   // array overload sends sizeof(packet) = 8
#else
  (void)thetaDeg;
  (void)valid;
  (void)rCm;
#endif
}

// Advances g_angleDeg one step, reversing and closing out the pass at an end.
static void advanceSweep() {
  float next = g_angleDeg + g_stepDeg;

  if (next > SWEEP_MAX_DEG || next < SWEEP_MIN_DEG) {
    ++g_sweepIndex;
    Serial.printf("[sweep %lu complete]\n", (unsigned long)g_sweepIndex);

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

  // The single stream. The head assigns its own stream ID; the Quest must use
  // that, not this registration position.
  tRadar = ardb->addTopic("a/radar", ARDBVisualType::Binary,
                          "Radar angle/range", RADAR_PACKET_BYTES);

  if (!tRadar.valid()) {
    Serial.println("[boot] WARNING: radar topic failed to register");
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

  publishSample(g_angleDeg, valid, rCm);

  if (valid) {
    Serial.printf("%6.1f deg  %7.1f cm\n", g_angleDeg, rCm);
  } else {
    Serial.printf("%6.1f deg        --\n", g_angleDeg);
  }

  advanceSweep();
}
