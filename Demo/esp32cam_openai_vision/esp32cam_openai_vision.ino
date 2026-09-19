/*
 * ESP32-CAM -> ARDB (MQTT) + OpenAI person detection
 *
 *   Core 1 (loop):  capture at 2 Hz -> publish JPEG + detection state to ARDB
 *   Core 0 (task):  at most every VISION_PERIOD_MS, ask OpenAI "is a person
 *                   visible?" plus a short scene description, over one
 *                   persistent (keep-alive) TLS connection
 *   Red LED:        blinks on every capture + send
 *   White flash:    ON while a human FACE (not just a body) is in frame
 *
 * Requires an arduino_secrets.h tab defining:
 *   SECRET_SSID, SECRET_PASS, SECRET_MQTT_HOST, SECRET_MQTT_PORT,
 *   SECRET_OPENAI_KEY
 *
 * Libraries: ArduinoJson v7.x, ArduinoMqttClient, ARDBClient
 * Board: AI Thinker ESP32-CAM | Partition: Huge APP | PSRAM: Enabled
 *
 * BISECTING: set ENABLE_ARDB or ENABLE_VISION to 0 to cut out that subsystem.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "arduino_secrets.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  FEATURE SWITCHES
// ============================================================
#define ENABLE_ARDB    1
#define ENABLE_VISION  1

#if ENABLE_ARDB
#include <ARDBClient.h>
#endif

// ============================================================
//  CONFIG  (secrets come from arduino_secrets.h)
// ============================================================
const char *WIFI_SSID = SECRET_SSID;
const char *WIFI_PASS = SECRET_PASS;

const char *MQTT_HOST = SECRET_MQTT_HOST;
const uint16_t MQTT_PORT = SECRET_MQTT_PORT;
const char *ARDB_CLIENT_ID = "esp32cam-01";

const char *OPENAI_KEY   = SECRET_OPENAI_KEY;
const char *OPENAI_HOST  = "api.openai.com";
const int   OPENAI_PORT  = 443;
const char *OPENAI_PATH  = "/v1/chat/completions";
const char *OPENAI_MODEL = "gpt-4.1-nano";   // fastest vision-capable model

// The \" sequences become literal backslash-quote in the outgoing JSON body.
const char *PROMPT =
    "Is a person visible? Is a human FACE clearly visible (eyes/nose/mouth, "
    "not just body or back of head)? JSON only: "
    "{\\\"human\\\":true or false,\\\"face\\\":true or false,\\\"desc\\\":\\\"max 4 words\\\"}";

const int MAX_TOKENS = 50;          // output tokens dominate latency

const uint32_t STREAM_PERIOD_MS = 500;    // 2 Hz to the broker
const uint32_t VISION_PERIOD_MS = 2000;   // min gap between OpenAI dispatches

#define FRAME_SIZE       FRAMESIZE_HQVGA   // 240x176: small = fast upload
#define JPEG_QUALITY     15                // higher number = smaller JPEG

// Red LED (GPIO33, active LOW) blinks on every capture + send.
// White flash (GPIO4) stays ON while a FACE is visible in frame.
#define FLASH_LED_PIN     4
#define STATUS_LED_PIN   33
const uint32_t BLINK_MS = 80;

const uint32_t CONNECT_TIMEOUT_MS = 15000;
const uint32_t FIRST_BYTE_TIMEOUT = 15000;
const uint32_t KEEPALIVE_MAX_MS   = 20000;   // reconnect if idle longer
const uint32_t READ_IDLE_TIMEOUT  = 15000;

#define DESC_MAX 96

// ============================================================
//  CAMERA PINS - AI Thinker
// ============================================================
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ============================================================
//  ARDB - constructed in setup(), NOT as globals.
//  Global constructors run before FreeRTOS is fully up and before
//  Serial.begin(), so a fault there gives a silent reset with no output.
// ============================================================
#if ENABLE_ARDB
static WiFiClient *ardbTransport = nullptr;
static ARDBConfig *ardbConfig    = nullptr;
static ARDBNetworkCallbacks *ardbNetwork = nullptr;
static ARDBClient *ardb          = nullptr;

static bool wifiIsConnected() { return WiFi.status() == WL_CONNECTED; }

static ARDBTopic tImage;   // 1
static ARDBTopic tHuman;   // 2
static ARDBTopic tDesc;    // 3
static ARDBTopic tLog;     // 4
static bool ardbReady = false;
#endif

// ============================================================
//  SHARED STATE
// ============================================================
static SemaphoreHandle_t stateMux = nullptr;
static SemaphoreHandle_t jobReady = nullptr;

static bool     g_human = false;
static bool     g_face  = false;
static char     g_desc[DESC_MAX] = "";
static uint32_t g_visionSeq = 0;
static bool     g_visionBusy = false;

static uint8_t *g_snapBuf = nullptr;
static size_t   g_snapLen = 0;

static bool cameraOk = false;

// Guarded lock helpers - never assert on a null handle.
static inline bool lockState() {
  if (!stateMux) return false;
  return xSemaphoreTake(stateMux, pdMS_TO_TICKS(1000)) == pdTRUE;
}
static inline void unlockState() {
  if (stateMux) xSemaphoreGive(stateMux);
}

// ============================================================
//  LED
// ============================================================
static void redLed(bool on)   { digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH); }
static void flashLed(bool on) { digitalWrite(FLASH_LED_PIN, on ? HIGH : LOW); }

// ============================================================
//  BASE64 - streamed straight into the TLS socket
// ============================================================
static const char B64_TABLE[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t base64EncodedLength(size_t rawLen) {
  return 4 * ((rawLen + 2) / 3);
}

static bool writeBase64(Client &client, const uint8_t *data, size_t len) {
  static char out[4096];              // vision task only; fewer, larger TLS records
  size_t o = 0, i = 0;

  while (i + 2 < len) {
    uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8) | data[i + 2];
    out[o++] = B64_TABLE[(v >> 18) & 0x3F];
    out[o++] = B64_TABLE[(v >> 12) & 0x3F];
    out[o++] = B64_TABLE[(v >> 6) & 0x3F];
    out[o++] = B64_TABLE[v & 0x3F];
    i += 3;
    if (o >= 4092) {
      if (client.write((const uint8_t *)out, o) != o) return false;
      o = 0;
    }
  }

  size_t rem = len - i;
  if (rem == 1) {
    uint32_t v = (uint32_t)data[i] << 16;
    out[o++] = B64_TABLE[(v >> 18) & 0x3F];
    out[o++] = B64_TABLE[(v >> 12) & 0x3F];
    out[o++] = '=';
    out[o++] = '=';
  } else if (rem == 2) {
    uint32_t v = ((uint32_t)data[i] << 16) | ((uint32_t)data[i + 1] << 8);
    out[o++] = B64_TABLE[(v >> 18) & 0x3F];
    out[o++] = B64_TABLE[(v >> 12) & 0x3F];
    out[o++] = B64_TABLE[(v >> 6) & 0x3F];
    out[o++] = '=';
  }
  if (o && client.write((const uint8_t *)out, o) != o) return false;
  return true;
}

// ============================================================
//  CAMERA
// ============================================================
static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size   = FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size   = FRAME_SIZE;
    config.jpeg_quality = 15;
    config.fb_count     = 1;
    config.fb_location  = CAMERA_FB_IN_DRAM;
  }

  // The 0x106 "not supported" probe failure is usually a marginal ribbon
  // seat or a sagging 3.3 V rail, and it is often intermittent, so retry.
  for (int attempt = 1; attempt <= 3; attempt++) {
    esp_err_t err = esp_camera_init(&config);
    if (err == ESP_OK) {
      sensor_t *s = esp_camera_sensor_get();
      if (s && s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
      }
      Serial.printf("[cam] ok (PID 0x%x) on attempt %d\n",
                    s ? s->id.PID : 0, attempt);
      return true;
    }
    Serial.printf("[cam] attempt %d failed: 0x%x\n", attempt, err);
    esp_camera_deinit();
    delay(500);
  }
  return false;
}

// ============================================================
//  NON-BLOCKING READ HELPERS
//  Explicit millis() deadlines everywhere. No Stream helpers: they honour
//  setTimeout(), which the ESP32 core interprets in SECONDS.
// ============================================================
static bool waitForData(WiFiClientSecure &c, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (c.available()) return true;
    if (!c.connected()) return c.available() > 0;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return false;
}

static bool readLine(WiFiClientSecure &c, String &line, uint32_t timeoutMs) {
  line = "";
  uint32_t last = millis();
  while (millis() - last < timeoutMs) {
    while (c.available()) {
      char ch = (char)c.read();
      if (ch == '\n') {
        if (line.endsWith("\r")) line.remove(line.length() - 1);
        return true;
      }
      line += ch;
      last = millis();
      if (line.length() > 4096) return true;
    }
    if (!c.connected() && !c.available()) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

static bool readExactly(WiFiClientSecure &c, String &dst, long n, uint32_t timeoutMs) {
  long got = 0;
  uint32_t last = millis();
  uint8_t buf[256];
  while (got < n) {
    int avail = c.available();
    if (avail > 0) {
      int want = (int)min((long)sizeof(buf), min((long)avail, n - got));
      int r = c.read(buf, want);
      if (r > 0) {
        for (int k = 0; k < r; k++) dst += (char)buf[k];
        got += r;
        last = millis();
      }
    } else {
      if (!c.connected()) return got >= n;
      if (millis() - last > timeoutMs) return false;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  return true;
}

static bool readResponse(WiFiClientSecure &client, int &status, String &body,
                         bool &keepAlive) {
  keepAlive = true;
  status = 0;
  body = "";
  body.reserve(1024);

  if (!waitForData(client, FIRST_BYTE_TIMEOUT)) {
    Serial.println("[http] no response before deadline");
    return false;
  }

  String line;
  if (!readLine(client, line, READ_IDLE_TIMEOUT)) return false;
  int sp = line.indexOf(' ');
  if (sp > 0) status = line.substring(sp + 1, sp + 4).toInt();

  bool chunked = false;
  long contentLength = -1;

  while (true) {
    if (!readLine(client, line, READ_IDLE_TIMEOUT)) return false;
    if (line.length() == 0) break;
    String lower = line;
    lower.toLowerCase();
    if (lower.startsWith("transfer-encoding:") && lower.indexOf("chunked") >= 0) {
      chunked = true;
    } else if (lower.startsWith("content-length:")) {
      contentLength = line.substring(line.indexOf(':') + 1).toInt();
    } else if (lower.startsWith("connection:") && lower.indexOf("close") >= 0) {
      keepAlive = false;
    }
  }

  if (chunked) {
    while (true) {
      if (!readLine(client, line, READ_IDLE_TIMEOUT)) break;
      line.trim();
      if (line.length() == 0) continue;
      long sz = strtol(line.c_str(), nullptr, 16);
      if (sz <= 0) {
        // Consume the trailer up to the blank line so the NEXT response on
        // this persistent connection starts clean.
        while (readLine(client, line, READ_IDLE_TIMEOUT) && line.length() > 0) {}
        break;
      }
      if (!readExactly(client, body, sz, READ_IDLE_TIMEOUT)) return false;
      if (!readLine(client, line, READ_IDLE_TIMEOUT)) keepAlive = false;
    }
  } else if (contentLength >= 0) {
    if (!readExactly(client, body, contentLength, READ_IDLE_TIMEOUT)) return false;
  } else {
    keepAlive = false;                 // body delimited by close
    uint32_t last = millis();
    while (millis() - last < READ_IDLE_TIMEOUT) {
      if (client.available()) { body += (char)client.read(); last = millis(); }
      else if (!client.connected()) break;
      else vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
  return true;
}

// ============================================================
//  OPENAI CALL (vision task only)
// ============================================================
// One TLS connection is kept open and reused: no DNS, TCP or TLS handshake
// per call. Touched only by the vision task.
static WiFiClientSecure *g_tls = nullptr;
static uint32_t g_tlsLastUse = 0;

static void tlsClose() {
  if (g_tls) g_tls->stop();
}

static bool tlsAlive() {
  return g_tls && g_tls->connected() && (millis() - g_tlsLastUse) < KEEPALIVE_MAX_MS;
}

static bool tlsOpen() {
  if (!g_tls) {
    g_tls = new WiFiClientSecure();
    if (!g_tls) return false;
  }
  g_tls->stop();
  g_tls->setInsecure();
  g_tls->setTimeout(15);            // SECONDS on the ESP32 core
  g_tls->setHandshakeTimeout(15);
  if (!g_tls->connect(OPENAI_HOST, OPENAI_PORT, CONNECT_TIMEOUT_MS)) return false;
  g_tls->setNoDelay(true);
  g_tlsLastUse = millis();
  return true;
}

static bool askOpenAI(const uint8_t *jpeg, size_t jpegLen,
                      bool &humanOut, bool &faceOut, char *descOut, size_t descCap) {
  uint32_t t0 = millis();

  String head = String("{\"model\":\"") + OPENAI_MODEL +
                "\",\"max_tokens\":" + MAX_TOKENS +
                ",\"response_format\":{\"type\":\"json_object\"}"
                ",\"messages\":[{\"role\":\"user\",\"content\":["
                "{\"type\":\"text\",\"text\":\"" + PROMPT + "\"},"
                "{\"type\":\"image_url\",\"image_url\":{\"url\":"
                "\"data:image/jpeg;base64,";
  // detail:low => OpenAI skips tiling and bills/processes a single 512px view
  String tail = "\",\"detail\":\"low\"}}]}]}";

  size_t total = head.length() + base64EncodedLength(jpegLen) + tail.length();

  // Headers + JSON head go out in a single write.
  String pre = String("POST ") + OPENAI_PATH + " HTTP/1.1\r\n" +
               "Host: " + OPENAI_HOST + "\r\n" +
               "Authorization: Bearer " + OPENAI_KEY + "\r\n" +
               "Content-Type: application/json\r\n" +
               "Content-Length: " + total + "\r\n" +
               "Connection: keep-alive\r\n\r\n" + head;

  int status = 0;
  String body;
  bool ok = false, keep = false;
  uint32_t tConn = t0, tSent = t0;

  // Attempt 1 may reuse a connection the server has silently dropped; if that
  // fails, attempt 2 retries once on a fresh connection.
  for (int attempt = 0; attempt < 2 && !ok; attempt++) {
    bool reused = tlsAlive();
    if (!reused && !tlsOpen()) {
      Serial.println("[vision] TLS connect failed");
      tlsClose();
      return false;
    }
    tConn = millis();
    WiFiClientSecure &c = *g_tls;

    bool sent = c.write((const uint8_t *)pre.c_str(), pre.length()) == pre.length() &&
                writeBase64(c, jpeg, jpegLen) &&
                c.write((const uint8_t *)tail.c_str(), tail.length()) == tail.length();
    // No flush(). On the ESP32 core flush() drains the RECEIVE buffer.
    if (!sent) {
      tlsClose();
      if (reused) { Serial.println("[vision] stale connection, retrying"); continue; }
      Serial.println("[vision] write failed");
      return false;
    }
    tSent = millis();

    ok = readResponse(c, status, body, keep);
    if (!ok) {
      tlsClose();
      if (reused) { Serial.println("[vision] stale connection, retrying"); continue; }
      Serial.printf("[vision] no usable response (HTTP %d)\n", status);
      return false;
    }
  }
  if (!ok) return false;

  uint32_t tDone = millis();
  if (keep) g_tlsLastUse = tDone; else tlsClose();

  JsonDocument filter;
  filter["choices"][0]["message"]["content"] = true;
  filter["error"]["message"] = true;

  JsonDocument doc;
  if (deserializeJson(doc, body, DeserializationOption::Filter(filter))) {
    Serial.println("[vision] outer JSON parse failed");
    Serial.println(body.substring(0, 300));
    return false;
  }

  const char *errMsg = doc["error"]["message"];
  if (errMsg) {
    Serial.printf("[vision] API HTTP %d: %s\n", status, errMsg);
    return false;
  }

  const char *content = doc["choices"][0]["message"]["content"];
  if (!content) {
    Serial.println("[vision] no content field");
    return false;
  }

  String inner(content);
  inner.replace("```json", "");
  inner.replace("```", "");
  inner.trim();

  JsonDocument res;
  if (deserializeJson(res, inner)) {
    Serial.printf("[vision] inner JSON parse failed: %s\n", inner.c_str());
    return false;
  }

  humanOut = res["human"] | false;
  faceOut  = humanOut && (res["face"] | false);
  const char *d = res["desc"] | "";
  strncpy(descOut, d, descCap - 1);
  descOut[descCap - 1] = '\0';

  Serial.printf("[vision] %lu ms (conn %lu, send %lu, api %lu) | human=%s face=%s | %s\n",
                (unsigned long)(millis() - t0),
                (unsigned long)(tConn - t0), (unsigned long)(tSent - tConn),
                (unsigned long)(tDone - tSent),
                humanOut ? "YES" : "no", faceOut ? "YES" : "no", descOut);
  return true;
}

// ============================================================
//  VISION TASK (core 0)
// ============================================================
#if ENABLE_VISION
static void visionTask(void *arg) {
  (void)arg;
  for (;;) {
    if (!jobReady) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
    if (xSemaphoreTake(jobReady, pdMS_TO_TICKS(1000)) != pdTRUE) {
      // Idle: keep the TLS connection warm so the next job skips the handshake.
      static uint32_t nextWarm = 0;
      if (WiFi.status() == WL_CONNECTED && !tlsAlive() &&
          (int32_t)(millis() - nextWarm) >= 0) {
        uint32_t tw = millis();
        bool w = tlsOpen();
        Serial.printf("[vision] warm-up TLS %s (%lu ms)\n", w ? "ok" : "FAILED",
                      (unsigned long)(millis() - tw));
        if (!w) { tlsClose(); nextWarm = millis() + 5000; }
      }
      continue;
    }

    uint8_t *buf = nullptr;
    size_t   len = 0;
    if (lockState()) {
      buf = g_snapBuf;
      len = g_snapLen;
      unlockState();
    }

    if (!buf || !len) {
      if (lockState()) { g_visionBusy = false; unlockState(); }
      continue;
    }

    bool human = false, face = false;
    char desc[DESC_MAX] = "";
    bool ok = askOpenAI(buf, len, human, face, desc, sizeof(desc));

    if (lockState()) {
      if (ok) {
        g_human = human;
        g_face  = face;
        strncpy(g_desc, desc, sizeof(g_desc) - 1);
        g_desc[sizeof(g_desc) - 1] = '\0';
        g_visionSeq++;
      }
      free(g_snapBuf);
      g_snapBuf = nullptr;
      g_snapLen = 0;
      g_visionBusy = false;
      unlockState();
    }
  }
}

static bool dispatchVision(camera_fb_t *fb) {
  if (!stateMux || !jobReady) return false;

  bool busy = true;
  if (lockState()) { busy = g_visionBusy; unlockState(); }
  if (busy) return false;    // retry on the next frame, not a full period later

  uint8_t *copy = (uint8_t *)ps_malloc(fb->len);
  if (!copy) {
    Serial.println("[vision] PSRAM alloc failed, skipping");
    return false;
  }
  memcpy(copy, fb->buf, fb->len);

  if (lockState()) {
    g_snapBuf    = copy;
    g_snapLen    = fb->len;
    g_visionBusy = true;
    unlockState();
    xSemaphoreGive(jobReady);
    return true;
  }
  free(copy);
  return false;
}
#endif

// ============================================================
//  SETUP
// ============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32-CAM -> ARDB + OpenAI ===");
  Serial.printf("[boot] heap %u, psram %s\n",
                (unsigned)ESP.getFreeHeap(), psramFound() ? "yes" : "no");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  redLed(false);
  flashLed(false);

  Serial.println("[boot] creating sync primitives");
  stateMux = xSemaphoreCreateMutex();
  jobReady = xSemaphoreCreateBinary();
  if (!stateMux || !jobReady) {
    Serial.println("[boot] FATAL: semaphore creation failed (out of heap)");
  }

  Serial.println("[boot] init camera");
  cameraOk = initCamera();
  if (!cameraOk) {
    Serial.println("[boot] camera unavailable. Check the ribbon cable seat "
                   "and the 3.3 V supply. Continuing without it.");
  }

  // Wi-Fi MUST be started before anything touches a socket. Opening a
  // WiFiClient while the TCP/IP stack is uninitialised trips
  // "assert failed: xQueueSemaphoreTake ... (pxQueue)" (null lwIP lock).
  // The sketch owns Wi-Fi; ARDB only gets a status callback.
  Serial.println("[boot] starting Wi-Fi");
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

#if ENABLE_ARDB
  Serial.println("[boot] constructing ARDB");
  ardbTransport = new WiFiClient();
  ardbConfig    = new ARDBConfig(ARDBConfig::wifiMqtt(
                      WIFI_SSID, WIFI_PASS, MQTT_HOST, ARDB_CLIENT_ID,
                      MQTT_PORT, /*retrySeconds=*/2, /*enabled=*/true));
  ardbNetwork   = new ARDBNetworkCallbacks(nullptr, wifiIsConnected);
  ardb          = new ARDBClient(*ardbTransport, *ardbNetwork, *ardbConfig);

  Serial.println("[boot] registering topics");
  tImage = ardb->addTopic("a/cam/img", ARDBVisualType::Binary,    "Camera JPEG");
  tHuman = ardb->addTopic("a/cam/h",   ARDBVisualType::ScalarF32, "Human present");
  tDesc  = ardb->addTopic("a/cam/d",   ARDBVisualType::Log,       "Scene description");
  tLog   = ardb->addTopic("a/cam/log", ARDBVisualType::Log,       "Device status");

  Serial.println("[boot] ardb.begin()");
  ardb->begin();
  ardbReady = true;
#endif

#if ENABLE_VISION
  Serial.println("[boot] starting vision task");
  xTaskCreatePinnedToCore(visionTask, "vision", 16384, nullptr, 1, nullptr, 0);
#endif

  Serial.printf("[boot] setup complete, heap %u\n", (unsigned)ESP.getFreeHeap());
}

// ============================================================
//  LOOP (core 1) - must never block
// ============================================================
void loop() {
  static uint32_t nextStream = 0;
  static uint32_t nextVision = 0;
  static uint32_t lastSeqSent = 0;
  static uint32_t nextHeartbeat = 0;
  static bool lastHuman = false;
  static bool firstPublish = true;
  static bool wasConnected = false;
  static uint32_t redOffAt = 0;
  static bool redOn = false;

  // Non-blocking end of the capture blink.
  if (redOn && (int32_t)(millis() - redOffAt) >= 0) {
    redLed(false);
    redOn = false;
  }

#if ENABLE_ARDB
  if (ardbReady && ardb) {
    ardb->update();

    bool nowConnected = ardb->connected();
    if (nowConnected && !wasConnected) {
      Serial.println("[ardb] connected");
      ardb->print(tLog, "esp32cam online");
    } else if (!nowConnected && wasConnected) {
      Serial.println("[ardb] disconnected");
    }
    wasConnected = nowConnected;
  }
#endif

  // Heartbeat so a silent board is distinguishable from a hung one.
  if ((int32_t)(millis() - nextHeartbeat) >= 0) {
    nextHeartbeat = millis() + 5000;
    Serial.printf("[hb] up %lus heap %u wifi %d",
                  (unsigned long)(millis() / 1000),
                  (unsigned)ESP.getFreeHeap(),
                  (int)WiFi.status());
#if ENABLE_ARDB
    if (ardb) Serial.printf(" ardb %d (state %d, err %d, sent %lu, dropped %lu)",
                            ardb->connected() ? 1 : 0, (int)ardb->state(),
                            ardb->lastConnectError(),
                            (unsigned long)ardb->sentPackets(),
                            (unsigned long)ardb->droppedPackets());
#endif
    Serial.println();
  }

  if ((int32_t)(millis() - nextStream) < 0) {
    delay(2);
    return;
  }
  nextStream = millis() + STREAM_PERIOD_MS;

  if (!cameraOk) return;

  redLed(true);                       // blink: picture taken
  redOn = true;
  redOffAt = millis() + BLINK_MS;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[cam] capture failed");
    return;
  }

  bool     human = false, face = false;
  uint32_t seq = 0;
  char     desc[DESC_MAX] = "";
  if (lockState()) {
    human = g_human;
    face  = g_face;
    seq   = g_visionSeq;
    strncpy(desc, g_desc, sizeof(desc));
    desc[sizeof(desc) - 1] = '\0';
    unlockState();
  }

  flashLed(face);                     // bright flash while a face is in frame

#if ENABLE_ARDB
  if (ardbReady && ardb && ardb->connected()) {
    ardb->printBytes(tImage, fb->buf, fb->len);
    ardb->print(tHuman, human ? 1.0f : 0.0f);

    // String() picks the documented text overload. Passing the char[96]
    // buffer directly would match the fixed-size-array overload and publish
    // all 96 bytes, trailing garbage included.
    if (desc[0] && (seq != lastSeqSent || human != lastHuman || firstPublish)) {
      ardb->print(tDesc, String(desc));
      lastSeqSent  = seq;
      lastHuman    = human;
      firstPublish = false;
    }
  }
#endif

#if ENABLE_VISION
  if ((int32_t)(millis() - nextVision) >= 0 && WiFi.status() == WL_CONNECTED) {
    if (dispatchVision(fb)) nextVision = millis() + VISION_PERIOD_MS;
  }
#endif

  esp_camera_fb_return(fb);
}