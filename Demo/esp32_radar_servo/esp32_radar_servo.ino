/*
 * ESP32 servo radar -> ARDB (MQTT)
 *
 *   A 360-degree POSITIONAL servo pans an HC-SR04 back and forth across an
 *   arc. Readings are batched and published to the ARDB head on ONE stream as
 *   a run of (bearing, range) pairs.
 *
 *   Servo signal  GPIO 32   (50 Hz LEDC, no servo library)
 *   Ultrasonic    GPIO 33 = TRIG (out), GPIO 25 = ECHO (in)
 *
 * !! ECHO IS 5 V ON A PLAIN HC-SR04 AND THE ESP32 IS NOT 5 V TOLERANT !!
 *   Use a 3.3 V part (HC-SR04P / RCWL-1601) or drop ECHO with a divider,
 *   e.g. 1 kOhm from ECHO to GPIO 25 and 2 kOhm from GPIO 25 to GND.
 *   Power the servo from its own 5 V supply with a common ground; it will
 *   brown out the ESP32 if run from the dev board's regulator.
 *
 * !! SET SERVO_RANGE_DEG TO MATCH YOUR SERVO !!
 *   360 for a 360-degree servo, 180 for an ordinary hobby servo. Wrong value
 *   means the sweep is the wrong size. SERVO_CALIBRATE 1 alternates the two
 *   arc endpoints so you can check the travel before running for real.
 *
 * Requires an arduino_secrets.h tab defining:
 *   SECRET_SSID, SECRET_PASS, SECRET_MQTT_HOST, SECRET_MQTT_PORT
 *
 * Libraries: ArduinoMqttClient, ARDBClient
 * Board: any ESP32 dev module (tested layout: ESP32-WROOM-32)
 *
 * BISECTING: set ENABLE_ARDB to 0 to sweep and print to Serial without the
 * ARDB client. (Wi-Fi still starts; see README "Open items".)
 */

#include <WiFi.h>
#include <string.h>

#include "arduino_secrets.h"

// ============================================================
//  FEATURE SWITCHES
// ============================================================
#define ENABLE_ARDB 1

// 0 = run normally
// 1 = trim neutral: the horn should sit perfectly still
// 2 = measure speed: one timed burst, then read off deg/s
#define SERVO_CALIBRATE 0

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
//  SERVO  (positional, 360-degree travel)
// ------------------------------------------------------------
// This is a POSITIONAL servo: the pulse width commands an ANGLE and the servo
// holds it. It is NOT continuous-rotation -- a CR servo spins while a pulse is
// applied and cannot hold a position at all. Driving this part with CR-style
// speed commands is what made it flick between two places instead of sweeping.
//
// SERVO_RANGE_DEG is the servo's FULL mechanical travel across the pulse band
// below. A "360 servo" means 360 here; an ordinary hobby servo means 180.
// GET THIS RIGHT: with 360 set on a 180-degree part the arc comes out twice as
// wide and hits the stops; with 180 set on a 360-degree part it comes out half
// as wide. If the sweep looks the wrong size, this is the constant to change.
static const float    SERVO_RANGE_DEG = 360.0f;
static const uint32_t SERVO_MIN_US = 500;    // pulse at 0 deg
static const uint32_t SERVO_MAX_US = 2500;   // pulse at SERVO_RANGE_DEG

// How fast the beam sweeps. One knob:
//     seconds per pass = arc / SWEEP_SPEED_DPS
//     ping spacing deg = SWEEP_SPEED_DPS * PING_PERIOD_MS / 1000
//   90 deg/s -> 2.0 s per pass, 5.4 deg spacing
//   60 deg/s -> 3.0 s per pass, 3.6 deg spacing
static const float SWEEP_SPEED_DPS = 90.0f;

static const uint32_t SERVO_FREQ_HZ = 50;
static const uint8_t  SERVO_RES_BITS = 16;
#if ESP_ARDUINO_VERSION_MAJOR < 3
static const uint8_t  SERVO_LEDC_CHANNEL = 0;  // core 2.x addresses a channel
#endif

// One-time move to the start of the arc before the first ping.
static const uint32_t SERVO_HOME_SETTLE_MS = 600;

// ------------------------------------------------------------
//  Sweep geometry
// ------------------------------------------------------------
static const float SWEEP_MIN_DEG = 0.0f;
static const float SWEEP_MAX_DEG = 180.0f;
static const float SWEEP_ARC_DEG = SWEEP_MAX_DEG - SWEEP_MIN_DEG;

// ------------------------------------------------------------
//  Ping cadence and angular resolution
// ------------------------------------------------------------
// Ping spacing is the thing that makes the plot look continuous or sparse:
//
//     spacing (deg) = SERVO_DPS * PING_PERIOD_MS / 1000
//
// PING_PERIOD_MS is already at the sensor's floor. The HC-SR04 datasheet asks
// for >= 60 ms between triggers so the previous burst has stopped ringing
// before the next one goes out; below that it starts answering with the old
// echo. So the only honest way to tighten spacing further is to lower
// SWEEP_SPEED_DPS, trading seconds per pass for degrees per reading.
//
//     90 deg/s -> 5.4 deg spacing, 2.0 s per pass
//     60 deg/s -> 3.6 deg spacing, 3.0 s per pass
//     45 deg/s -> 2.7 deg spacing, 4.0 s per pass
static const uint32_t PING_PERIOD_MS = 60;

// Task tick. Only has to be fine enough to hit PING_PERIOD_MS on time -- the
// servo is NOT re-commanded on this tick. It runs at a constant speed and is
// only written at a reversal, so the motion is genuinely continuous rather
// than a fast sequence of steps.
static const uint32_t RADAR_TICK_MS = 10;

// How often loop() drains the batch and publishes. Matching the head's 100 ms
// delta interval means every packet we send is one the Quest can receive.
// Defined outside ENABLE_ARDB because loop() paces its Serial output with it
// either way.
static const uint32_t PACKET_PERIOD_MS = 100;

// ------------------------------------------------------------
//  Ultrasonic ranging
// ------------------------------------------------------------
// 0.0343 cm/us at ~20 C, halved because the burst makes a round trip.
static const float CM_PER_US = 0.0343f / 2.0f;

static const float MIN_RANGE_CM = 3.0f;    // below this the module lies
static const float MAX_RANGE_CM = 200.0f;  // trim to the room you are testing in

// pulseIn() bounds the whole wait. Must stay well under PING_PERIOD_MS.
static const uint32_t ECHO_TIMEOUT_US =
    (uint32_t)((MAX_RANGE_CM / CM_PER_US) * 1.2f);
static_assert(ECHO_TIMEOUT_US < PING_PERIOD_MS * 1000UL,
              "echo timeout must fit inside one ping period");

static const uint32_t WIFI_DOWN_RECONNECT_MS = 10000;

// ============================================================
//  ARDB WIRE SCHEMA
// ============================================================
// ARDB metadata carries a type byte and a byte count. It does NOT carry field
// order, units, or byte order, so those are fixed here and must be mirrored by
// the Quest decoder. See ARDBClient/Quest_Type_Report.md.
//
// Frame: theta is measured counter-clockwise from the sensor's right-hand axis
// in the horizontal plane, so 0 points right, 90 straight ahead, 180 left.
//
// ONE stream. Each packet is a RUN of consecutive readings, oldest first:
//
//   a/radar   Binary (255)   8..64 bytes, always a multiple of 8
//
//     N x float[2] { thetaDeg, rCm },  N = payloadLength / 8,  1 <= N <= 8
//
//     thetaDeg  bearing, 0..180
//     rCm       range in centimetres from the sensor face
//
//     rCm == 0.0 is the NO-ECHO sentinel: the beam swept that bearing and
//     nothing answered. It is not a target at the sensor's origin. Zero is
//     safe as a sentinel because a real reading is always >= MIN_RANGE_CM.
//     A decoder that plots rCm without testing for zero will draw a false
//     contact at the origin on every empty bearing.
//
//     All floats are IEEE-754 binary32 LITTLE-ENDIAN (the ESP32's in-memory
//     layout, copied verbatim by ARDBClient).
//
// Batching is what makes the plot continuous. The head emits deltas no more
// often than every 100 ms, so a one-point packet caps the Quest at 10 readings
// per second no matter how fast the sensor runs. Packing every reading taken
// since the last publish into one packet delivers all of them -- ~16.7/s at
// PING_PERIOD_MS 60 -- through that same 10 Hz channel.
//
// A single-point packet is byte-identical to the previous one-reading format,
// so a decoder written for that still works; it just sees N = 1.

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

// ARDBClient's per-topic gate restarts from the moment a publish *finishes*,
// so a 10 Hz limit against a 100 ms cadence drops a packet whenever a publish
// takes more than 0 ms -- which is always. Give the client headroom and let
// the head's 100 ms delta interval be the real pacer.
static const uint16_t ARDB_PUBLISH_RATE_HZ = 25;
#endif

// One reading on the wire. Packed explicitly rather than trusting the default
// layout, because the type guide is emphatic that a struct's natural layout is
// not a wire schema.
struct __attribute__((packed)) RadarPoint {
  float thetaDeg;
  float rCm;
};
static_assert(sizeof(float) == 4, "radar packet assumes 32-bit float");
static_assert(sizeof(RadarPoint) == 8, "radar point must be exactly 8 bytes");

// 8 x 8 = 64 bytes, exactly the head's ordinary payload cap.
static const size_t RADAR_MAX_POINTS = 8;
static_assert(RADAR_MAX_POINTS * sizeof(RadarPoint) <= 64,
              "batch must fit the ARDB ordinary payload limit");

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

// Commands an absolute bearing. The pulse spans SERVO_RANGE_DEG, so sweeping
// 0..180 on a 360-degree servo uses the lower half of the pulse band -- which
// is correct, and why SERVO_RANGE_DEG has to match the actual part.
static void servoWriteDegrees(float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > SERVO_RANGE_DEG) deg = SERVO_RANGE_DEG;

  const float t = deg / SERVO_RANGE_DEG;
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
//  SHARED STATE
// ============================================================
// The sweep runs in its own task on core 0. loop() runs on core 1 and only
// drains finished readings through this mutex, so a blocking network call in
// loop() can never stall the servo. That matters more than it sounds: with the
// broker unreachable, ARDBClient retries every 2 s and each attempt sits in a
// TCP connect whose default timeout is 3000 ms (WIFI_CLIENT_DEF_CONN_TIMEOUT_MS
// in the ESP32 core). Sharing one thread with that is what made the sweep crawl.
static SemaphoreHandle_t g_sampleMux = nullptr;
static TaskHandle_t      g_radarTask = nullptr;

static RadarPoint g_points[RADAR_MAX_POINTS];
static size_t     g_pointCount = 0;
static uint32_t   g_droppedPoints = 0;   // buffer overruns, for the Serial log
static uint32_t   g_sweepCount = 0;      // completed passes

// Appends one reading, dropping the OLDEST if loop() has fallen behind: on a
// live plot the newest bearings are the ones worth keeping.
static void pushPoint(float thetaDeg, float rCm, uint32_t sweeps) {
  if (!g_sampleMux) return;
  if (xSemaphoreTake(g_sampleMux, pdMS_TO_TICKS(5)) != pdTRUE) return;

  if (g_pointCount == RADAR_MAX_POINTS) {
    memmove(&g_points[0], &g_points[1],
            sizeof(RadarPoint) * (RADAR_MAX_POINTS - 1));
    g_pointCount = RADAR_MAX_POINTS - 1;
    ++g_droppedPoints;
  }
  g_points[g_pointCount].thetaDeg = thetaDeg;
  g_points[g_pointCount].rCm = rCm;
  ++g_pointCount;
  g_sweepCount = sweeps;

  xSemaphoreGive(g_sampleMux);
}

// ============================================================
//  RADAR TASK  (core 0)
// ============================================================
static void radarTask(void *) {
  float angle = SWEEP_MIN_DEG;
  float dir = 1.0f;
  uint32_t sweeps = 0;

  // Park at the start of the arc and let the horn get there before the first
  // ping, or reading 1 is taken somewhere mid-travel.
  servoWriteDegrees(angle);
  vTaskDelay(pdMS_TO_TICKS(SERVO_HOME_SETTLE_MS));

  uint32_t lastMove = millis();
  uint32_t nextPing = millis();

  for (;;) {
    const uint32_t now = millis();

    // Integrate position from elapsed time rather than a fixed step, so speed
    // stays honest when an iteration runs late. Every tick commands a new
    // angle, 0.9 deg apart at 90 deg/s, which reads as continuous motion.
    uint32_t dt = now - lastMove;
    lastMove = now;
    if (dt > RADAR_TICK_MS * 4) dt = RADAR_TICK_MS * 4;  // no lurch after a stall

    angle += dir * SWEEP_SPEED_DPS * (float)dt / 1000.0f;
    if (angle >= SWEEP_MAX_DEG) {
      angle = SWEEP_MAX_DEG;
      dir = -1.0f;
      ++sweeps;
    } else if (angle <= SWEEP_MIN_DEG) {
      angle = SWEEP_MIN_DEG;
      dir = 1.0f;
      ++sweeps;
    }
    servoWriteDegrees(angle);

    if ((int32_t)(now - nextPing) >= 0) {
      nextPing += PING_PERIOD_MS;
      if ((int32_t)(now - nextPing) > 0) nextPing = now + PING_PERIOD_MS;

      // Bearing latched BEFORE the ping: the horn keeps moving during the
      // up-to-14 ms pulseIn, about 1.3 deg at 90 deg/s.
      const float bearing = angle;
      float rCm = 0.0f;
      const bool valid = sonarPingCm(rCm);
      pushPoint(bearing, valid ? rCm : 0.0f, sweeps);
    }

    // Yields core 0 to its idle task. Without this the task watchdog trips.
    vTaskDelay(pdMS_TO_TICKS(RADAR_TICK_MS));
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("=== ESP32 servo radar (360 servo) -> ARDB ===");
  Serial.printf("[boot] servo=GPIO%d trig=GPIO%d echo=GPIO%d\n", SERVO_PIN,
                TRIG_PIN, ECHO_PIN);

  sonarBegin();
  servoBegin();

#if SERVO_CALIBRATE == 1
  // ---- Endpoint check -----------------------------------------------
  Serial.println();
  Serial.println("=== CALIBRATE 1: endpoint check ===");
  Serial.printf("Alternating %.0f and %.0f deg every 2 s.\n",
                SWEEP_MIN_DEG, SWEEP_MAX_DEG);
  Serial.printf("SERVO_RANGE_DEG is %.0f. If the horn moves the wrong amount,\n",
                SERVO_RANGE_DEG);
  Serial.println("that constant does not match the servo -- try 180 or 360.");
  return;

#else
  // ---- Normal run ---------------------------------------------------
  Serial.printf("[boot] arc %.0f..%.0f deg on a %.0f deg servo, %.0f deg/s\n",
                SWEEP_MIN_DEG, SWEEP_MAX_DEG, SERVO_RANGE_DEG, SWEEP_SPEED_DPS);
  Serial.printf("[boot] ping %lu ms -> %.1f deg spacing, %.1f s per pass\n",
                (unsigned long)PING_PERIOD_MS,
                SWEEP_SPEED_DPS * (float)PING_PERIOD_MS / 1000.0f,
                SWEEP_ARC_DEG / SWEEP_SPEED_DPS);

  wifiStart();

#if ENABLE_ARDB
  Serial.println("[boot] constructing ARDB");
  ardbTransport = new WiFiClient();
  // Default is 3000 ms. ARDBClient's mqttConnectTimeoutMs only bounds the
  // CONNACK wait, not the TCP connect underneath it, so with no broker on the
  // network loop() would otherwise block for 3 s on every retry.
  ardbTransport->setConnectionTimeout(250);
  ardbConfig = new ARDBConfig(ARDBConfig::wifiMqtt(
      WIFI_SSID, WIFI_PASS, MQTT_HOST, ARDB_CLIENT_ID, MQTT_PORT));
  // beginNetwork is null: the sketch already owns Wi-Fi, ARDB only polls it.
  ardbNetwork = new ARDBNetworkCallbacks(nullptr, wifiIsConnected);
  ardb = new ARDBClient(*ardbTransport, *ardbNetwork, *ardbConfig,
                        ARDB_PUBLISH_RATE_HZ);

  // Variable length: a packet carries however many readings were taken since
  // the last publish, so 0 rather than a fixed byte count.
  tRadar = ardb->addTopic("a/radar", ARDBVisualType::Binary,
                          "Radar angle/range", 0);

  if (!tRadar.valid()) {
    Serial.println("[boot] WARNING: radar topic failed to register");
  }

  Serial.println("[boot] ardb.begin()");
  ardb->begin();
#endif

  g_sampleMux = xSemaphoreCreateMutex();
  if (!g_sampleMux) {
    Serial.println("[boot] FATAL: mutex alloc failed");
    return;   // no task: the sweep never starts, and Serial says why
  }

  // Core 0. loop() owns core 1 and everything that can block on the network.
  if (xTaskCreatePinnedToCore(radarTask, "radar", 4096, nullptr, 2,
                              &g_radarTask, 0) != pdPASS) {
    Serial.println("[boot] FATAL: radar task failed to start");
  }
#endif  // SERVO_CALIBRATE
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
#if SERVO_CALIBRATE != 0
  servoWriteDegrees(SWEEP_MIN_DEG);
  Serial.printf("commanded %.0f deg\n", SWEEP_MIN_DEG);
  delay(2000);
  servoWriteDegrees(SWEEP_MAX_DEG);
  Serial.printf("commanded %.0f deg\n", SWEEP_MAX_DEG);
  delay(2000);
  return;
#else

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

  // Drain on the head's own cadence. Publishing faster would only produce
  // packets it throttles away; slower would let the batch overflow.
  static uint32_t nextPacketAt = 0;
  const uint32_t now = millis();
  if ((int32_t)(now - nextPacketAt) < 0) {
    delay(2);
    return;
  }
  nextPacketAt = now + PACKET_PERIOD_MS;

  RadarPoint batch[RADAR_MAX_POINTS];
  size_t     n = 0;
  uint32_t   sweeps = 0, dropped = 0;

  // Zero timeout: if the task holds the mutex we simply retry next cycle
  // rather than waiting on it.
  if (g_sampleMux && xSemaphoreTake(g_sampleMux, 0) == pdTRUE) {
    n = g_pointCount;
    if (n) memcpy(batch, g_points, sizeof(RadarPoint) * n);
    g_pointCount = 0;
    sweeps = g_sweepCount;
    dropped = g_droppedPoints;
    xSemaphoreGive(g_sampleMux);
  }

  if (n) {
#if ENABLE_ARDB
    // One packet, N readings, oldest first.
    ardb->printBytes(tRadar, batch, n * sizeof(RadarPoint));
#endif
    for (size_t i = 0; i < n; i++) {
      if (batch[i].rCm > 0.0f) {
        Serial.printf("%6.1f deg  %7.1f cm\n", batch[i].thetaDeg, batch[i].rCm);
      } else {
        Serial.printf("%6.1f deg        --\n", batch[i].thetaDeg);
      }
    }
  }

  static uint32_t lastSweep = 0;
  if (sweeps != lastSweep) {
    lastSweep = sweeps;
    Serial.printf("[sweep %lu complete] batch %u, dropped %lu\n",
                  (unsigned long)sweeps, (unsigned)n, (unsigned long)dropped);
  }
#endif  // SERVO_CALIBRATE
}
