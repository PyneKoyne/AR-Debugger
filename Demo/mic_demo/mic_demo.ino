/*
 * mic_demo - I2S microphone audio spectrum analysis streamed to ARDB
 *
 *   loop():        capture 512 samples at 16 kHz, run a 512-point real FFT,
 *                  publish a 64-band spectrum plus lumped band levels
 *   netSupervisor: repairs Wi-Fi instead of waiting on it, reboots as a last
 *                  resort
 *
 * Board:   ESP32 or ESP32-S3 with an I2S MEMS mic (INMP441 / ICS-43434 /
 *          SPH0645). Uses the legacy driver/i2s.h API, which is still shipped
 *          by ESP32 Arduino core 2.x and 3.x (it emits a deprecation warning).
 *
 * Requires an arduino_secrets.h tab defining:
 *   SECRET_SSID, SECRET_PASS, SECRET_MQTT_HOST, SECRET_MQTT_PORT
 * Copy arduino_secrets.example.h to arduino_secrets.h to create it.
 *
 * Libraries: arduinoFFT (v2.x), ArduinoMqttClient, ARDBClient
 *
 * BISECTING: set ENABLE_ARDB or ENABLE_MIC to 0 to cut out that subsystem.
 *
 * ---------------------------------------------------------------------------
 * Why 64 bands and not 256
 * ---------------------------------------------------------------------------
 * The ARDB head caps an application payload at 64 bytes
 * (kMaxApplicationPayloadBytes) and the protocol has no fragmentation. One
 * uint8 per band is therefore the highest band count that fits in a single
 * message: 64 bands x 1 byte = 64 bytes exactly. A 256-bin linear spectrum
 * cannot be sent, so the 256 usable bins are folded into 64 bands weighted
 * towards log spacing, which is also how audio spectra are normally displayed.
 *
 * ---------------------------------------------------------------------------
 * ARDB stream contract (little-endian; ESP32 native byte order)
 * ---------------------------------------------------------------------------
 * The type byte selects a Quest decoder; units, order and scale are a
 * publisher/Quest agreement that ARDB metadata does NOT carry. This sketch's
 * agreement is:
 *
 *  m/aud/s  Binary (64 B) SPECTRUM. 64 unsigned bytes, band 0 = lowest
 *           frequency, band 63 = highest. Decode each byte to decibels with
 *
 *               dBFS = -78.0 + (byte / 255.0) * 78.0
 *
 *           so 0 means "at or below -78 dBFS" and 255 means 0 dBFS (full
 *           scale). One LSB is 0.306 dB.
 *
 *           Band edges target a log sweep from 60 Hz to 8000 Hz,
 *               fLow(i) = 60 * (8000/60) ^ (i / 64)
 *           but each edge is then quantised to an FFT bin and forced to be
 *           contiguous and at least one bin wide. Because the bin width is
 *           31.25 Hz, the target log steps below ~1.3 kHz are narrower than a
 *           single bin, so the ACTUAL layout is:
 *
 *               bands  0..39  one bin each  ->  62.5 Hz .. 1312 Hz, linear
 *               bands 40..63  2..19 bins    ->  1312 Hz .. 8000 Hz, log
 *
 *           Together they cover bins 2..255, i.e. 62.5 Hz to 8000 Hz with no
 *           gaps and no overlap. Treat the axis as non-uniform: use the edge
 *           table rather than assuming a uniform or purely log axis. The exact
 *           per-band edges are printed to Serial at boot.
 *
 *           Plot as a 64-bucket bar chart, or as one column of a scrolling
 *           spectrogram heatmap.
 *
 *  Only the spectrum and the log are published. Bass/mid/treble band levels,
 *  the interpolated peak frequency and the broadband RMS level are all still
 *  computed and printed to Serial, but they are not streamed: the first two
 *  are derivable from the spectrum, and the head's 8-stream budget is shared
 *  across every publisher on the broker, so a slot is worth more.
 *
 *  Note the peak frequency is the one value NOT fully recoverable from the
 *  published spectrum. It is interpolated across 31.25 Hz bins, whereas the
 *  top spectrum bands are up to 19 bins (594 Hz) wide. If you need precise
 *  pitch on the Quest side rather than in the serial log, re-add it as a
 *  ScalarF32 stream.
 *
 *  m/aud/l  Log (variable) UTF-8 status text, no trailing NUL, <= 64 bytes.
 * ---------------------------------------------------------------------------
 */

#include <driver/i2s.h>
#include <WiFi.h>

#include "arduino_secrets.h"
#include "arduinoFFT.h"

// ============================================================
//  FEATURE SWITCHES
// ============================================================
#define ENABLE_ARDB 1
#define ENABLE_MIC  1

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
const char* ARDB_CLIENT_ID = "demo-mic-01";

// I2S pins. GPIO 6-11 are wired to the SPI flash on the classic ESP32, so the
// 11/15/10 assignment only works on the ESP32-S3/S2.
#define I2S_SCK 11  // mic SCK / BCLK
#define I2S_WS  15  // mic WS / LRCL
#define I2S_SD  10  // mic SD / DOUT
#define I2S_PORT I2S_NUM_0

// FFT configuration. samples must be a power of two.
// Single precision throughout: the ESP32's FPU is single-precision only, so
// every double operation is emulated in software. Running the FFT in float
// keeps it on the hardware FPU and halves the working set.
const uint16_t samples = 512;
const float samplingFrequency = 16000.0f;
const float binHz = samplingFrequency / samples;  // 31.25 Hz

// Spectrum band layout. 64 bands x 1 byte = the 64-byte payload ceiling.
const uint8_t kBandCount = 64;
const float kSpectrumLowHz = 60.0f;
const float kSpectrumHighHz = 8000.0f;  // Nyquist for 16 kHz

// dB window used to quantise each band into a byte.
const float kFloorDb = -78.0f;
const float kCeilDb = 0.0f;

// Temporal smoothing: fast attack, slower decay, which reads much better on a
// plot than raw frame-to-frame values.
const float kAttack = 0.65f;
const float kDecay = 0.25f;

// A 24-bit left-justified sample in a 32-bit I2S word: >> 8 recovers the
// signed 24-bit value, and 2^23 is its full scale. Stored as a reciprocal so
// the per-sample conversion is a multiply rather than a divide.
const float kInvFullScale24 = 1.0f / 8388608.0f;

const uint32_t SERIAL_PLOT_PERIOD_MS = 200;
const uint32_t HEARTBEAT_PERIOD_MS = 5000;

// Self-healing (see netSupervisor). A successful ARDB publish is this
// sketch's proof of end-to-end liveness, the way a completed HTTP response
// is in esp32cam_openai_vision.
const uint32_t WIFI_DOWN_RECONNECT_MS = 30000;  // link down this long -> re-join
const uint32_t FORCE_RECONNECT_GAP_MS = 30000;  // min gap between forced re-joins
const uint32_t PUBLISH_STALE_MS = 30000;        // link up but nothing sent -> re-join
const uint32_t REBOOT_AFTER_MS = 180000;        // nothing sent at all -> reboot

// ============================================================
//  BUFFERS  (globals, not stack: loopTask has an 8 KB stack by default)
// ============================================================
static float vReal[samples];
static float vImag[samples];
static int32_t sampleBuffer[samples];
static float amp[samples / 2];

ArduinoFFT<float> FFT =
    ArduinoFFT<float>(vReal, vImag, samples, samplingFrequency);

// Band -> FFT bin mapping, built once in setup().
static uint16_t bandBinLo[kBandCount];
static uint16_t bandBinHi[kBandCount];  // exclusive
static float bandLowHz[kBandCount];
static float bandHighHz[kBandCount];

// Smoothed band levels in dB, and the payload built from them.
static float bandDb[kBandCount];
static uint8_t spectrumPayload[kBandCount];

static bool i2sOk = false;

// ============================================================
//  ARDB - constructed in setup(), NOT as globals.
//  Global constructors run before FreeRTOS is fully up and before
//  Serial.begin(), so a fault there gives a silent reset with no output.
// ============================================================
#if ENABLE_ARDB
static WiFiClient* ardbTransport = nullptr;
static ARDBConfig* ardbConfig = nullptr;
static ARDBNetworkCallbacks* ardbNetwork = nullptr;
static ARDBClient* ardb = nullptr;

static bool wifiIsConnected() { return WiFi.status() == WL_CONNECTED; }

static ARDBTopic tSpectrum;  // 1
static ARDBTopic tLog;       // 2
static bool ardbReady = false;
#endif

// ============================================================
//  SHARED STATE
// ============================================================
// Network health, read by netSupervisor().
static uint32_t g_lastPublishOk = 0;  // last successful ARDB publish

// Set from the Wi-Fi event callback and drained in loop().
static volatile bool g_wifiGotIpEvent = false;
static volatile bool g_wifiLostEvent = false;
static volatile int g_wifiLostReason = 0;

// ============================================================
//  BAND TABLE
// ============================================================
// Builds contiguous, non-overlapping, monotonic band -> bin ranges. Log band
// edges below ~1.3 kHz are narrower than the 31.25 Hz bin width, so the low
// bands are forced to one bin each instead of aliasing onto the same bin.
void buildBandTable() {
  const uint16_t minBin = 2;            // skip DC and the first bin
  const uint16_t maxBin = samples / 2;  // exclusive, 256
  const float ratio = kSpectrumHighHz / kSpectrumLowHz;

  uint16_t cursor = minBin;
  for (uint8_t i = 0; i < kBandCount; i++) {
    const float fLo = kSpectrumLowHz * powf(ratio, (float)i / kBandCount);
    const float fHi = kSpectrumLowHz * powf(ratio, (float)(i + 1) / kBandCount);

    uint16_t lo = (uint16_t)lroundf(fLo / (float)binHz);
    uint16_t hi = (uint16_t)lroundf(fHi / (float)binHz);

    if (lo < cursor) lo = cursor;
    if (hi <= lo) hi = lo + 1;

    // Reserve one bin per remaining band so the table never runs off the end.
    const uint16_t remaining = kBandCount - i - 1;
    if (lo + 1 + remaining > maxBin) {
      lo = (uint16_t)(maxBin - 1 - remaining);
      hi = (uint16_t)(lo + 1);
    }
    if (hi + remaining > maxBin) {
      hi = (uint16_t)(maxBin - remaining);
    }
    if (hi <= lo) hi = lo + 1;

    bandBinLo[i] = lo;
    bandBinHi[i] = hi;
    bandLowHz[i] = lo * (float)binHz;
    bandHighHz[i] = hi * (float)binHz;
    cursor = hi;
  }
}

void printBandTable() {
  Serial.println(F("[fft] band : bins : Hz range  (the plot's x axis)"));
  for (uint8_t i = 0; i < kBandCount; i++) {
    Serial.printf("  %2u : %3u-%3u : %7.1f - %7.1f\n", i, bandBinLo[i],
                  bandBinHi[i] - 1, bandLowHz[i], bandHighHz[i]);
  }
}

// ============================================================
//  I2S
// ============================================================
bool initI2s() {
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = (uint32_t)samplingFrequency,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      // 4 x 256 = 1024 samples buffered, two full FFT blocks. The original
      // 8 x 64 held exactly one block, so any jitter caused an overrun.
      .dma_buf_count = 4,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  const i2s_pin_config_t pin_config = {.bck_io_num = I2S_SCK,
                                       .ws_io_num = I2S_WS,
                                       .data_out_num = I2S_PIN_NO_CHANGE,
                                       .data_in_num = I2S_SD};

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("[i2s] driver_install failed: 0x%x\n", err);
    return false;
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("[i2s] set_pin failed: 0x%x\n", err);
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  i2s_start(I2S_PORT);
  Serial.printf("[i2s] ok  sck=%d ws=%d sd=%d  %u Hz\n", I2S_SCK, I2S_WS,
                I2S_SD, (unsigned)samplingFrequency);
  return true;
}

// ============================================================
//  DSP HELPERS
// ============================================================
inline float amplitudeToDb(float amplitude) {
  // 1e-9 keeps log10f finite on a silent block without visibly shifting the
  // result, since the floor is -78 dB.
  return 20.0f * log10f(amplitude + 1e-9f);
}

inline uint8_t dbToByte(float db) {
  float t = (db - kFloorDb) / (kCeilDb - kFloorDb);
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return (uint8_t)lroundf(t * 255.0f);
}

// Energy-summed amplitude across an inclusive-exclusive bin range.
float bandAmplitude(const float* magnitudes, uint16_t lo, uint16_t hi) {
  float power = 0.0f;
  for (uint16_t b = lo; b < hi; b++) {
    power += magnitudes[b] * magnitudes[b];
  }
  return sqrtf(power);
}

// ============================================================
//  WI-FI  (owned by the sketch; ARDB only gets a status callback)
// ============================================================
// Disconnect reason codes worth knowing:
//   200 BEACON_TIMEOUT (AP too weak / radio starved)   201 NO_AP_FOUND
//   2 AUTH_EXPIRE   15 4WAY_HANDSHAKE_TIMEOUT   202 AUTH_FAIL   8 ASSOC_LEAVE
static void onWifiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      g_wifiLostReason = (int)info.wifi_sta_disconnected.reason;
      g_wifiLostEvent = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      g_wifiGotIpEvent = true;
      break;
    default:
      break;
  }
}

static void wifiJoin() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static void wifiStart() {
  WiFi.onEvent(onWifiEvent);
  WiFi.persistent(false);  // do not rewrite flash on every join
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  // Default is FAST_SCAN: join the FIRST matching AP. On a multi-AP network
  // that is often a weak one. Scan every channel and take the strongest.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
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
    WiFi.disconnect(false, false);
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
  Serial.println(F("=== mic_demo -> ARDB audio spectrum ==="));
  Serial.printf("[boot] heap %u\n", (unsigned)ESP.getFreeHeap());

  Serial.println(F("[boot] building band table"));
  buildBandTable();
  printBandTable();
  for (uint8_t i = 0; i < kBandCount; i++) {
    bandDb[i] = kFloorDb;
  }

#if ENABLE_MIC
  Serial.println(F("[boot] init I2S"));
  i2sOk = initI2s();
  if (!i2sOk) {
    Serial.println(F("[boot] microphone unavailable. Check the SCK/WS/SD pins "
                     "and the 3V3 rail. Continuing without it."));
  }
#endif

  // Wi-Fi MUST be started before anything touches a socket. Opening a
  // WiFiClient while the TCP/IP stack is uninitialised trips
  // "assert failed: xQueueSemaphoreTake ... (pxQueue)" (null lwIP lock).
  // The sketch owns Wi-Fi; ARDB only gets a status callback.
  Serial.println(F("[boot] starting Wi-Fi"));
  wifiStart();

#if ENABLE_ARDB
  Serial.println(F("[boot] constructing ARDB"));
  ardbTransport = new WiFiClient();
  ardbConfig = new ARDBConfig(ARDBConfig::wifiMqtt(
      WIFI_SSID, WIFI_PASS, MQTT_HOST, ARDB_CLIENT_ID, MQTT_PORT,
      /* retrySeconds */ 3, /* enabled */ true));
  // ARDBClient copies the config, so every field must be final BEFORE the
  // constructor runs. connect() blocks waiting for CONNACK; keep it short.
  ardbConfig->mqttConnectTimeoutMs = 600;
  // beginNetwork is null: the sketch owns association, ARDB just waits for
  // the link and then owns MQTT retries.
  ardbNetwork = new ARDBNetworkCallbacks(nullptr, wifiIsConnected);
  // One FFT block is 32 ms, so the loop produces ~31 spectra/s. 10 Hz on the
  // wire matches the head's 100 ms delta floor; the rest are dropped by the
  // rate limiter rather than queued.
  ardb = new ARDBClient(*ardbTransport, *ardbNetwork, *ardbConfig,
                        /* dataPublishRateHz */ 10);

  Serial.println(F("[boot] registering topics"));
  // Registration order fixes the metadata ids ardb/meta/<clientId>/1..2.
  // Keep it stable; clear retained metadata if a topic is renamed.
  // Display names must stay <= 48 bytes or the head rejects the descriptor.
  tSpectrum = ardb->addTopic("m/aud/s", ARDBVisualType::Binary,
                             "Audio spectrum 64 bands 62Hz-8kHz", kBandCount);
  tLog = ardb->addTopic("m/aud/l", ARDBVisualType::Log, "Mic demo log");

  if (!tSpectrum.valid() || !tLog.valid()) {
    Serial.println(F("[ardb] FATAL: topic registration rejected"));
  }

  Serial.println(F("[boot] ardb->begin()"));
  ardb->begin();
  ardbReady = true;
#endif

  g_lastPublishOk = millis();  // start the connectivity clock at boot
  Serial.printf("[boot] setup complete, heap %u\n", (unsigned)ESP.getFreeHeap());
}

// ============================================================
//  LOOP - must never block
// ============================================================
void loop() {
  static uint32_t nextPlotMs = 0;
  static uint32_t nextHeartbeat = 0;
  static bool wasConnected = false;
  static float lastPeakHz = 0.0f;
  static float lastLevelDb = kFloorDb;
  static float lastBass = kFloorDb;
  static float lastMid = kFloorDb;
  static float lastTreble = kFloorDb;

#if ENABLE_ARDB
  if (ardbReady && ardb) {
    ardb->update();  // drives Wi-Fi status, MQTT and metadata

    const bool nowConnected = ardb->connected();
    if (nowConnected && !wasConnected) {
      Serial.println(F("[ardb] connected"));
      // Text belongs on the variable-length Log topic; a fixed-size topic
      // would reject it on the length check.
      ardb->print(tLog, i2sOk ? "mic online 64 bands 62Hz-8kHz"
                              : "mic online (no I2S)");
    } else if (!nowConnected && wasConnected) {
      Serial.println(F("[ardb] disconnected"));
    }
    wasConnected = nowConnected;
  }
#endif

  reportWifiEvents();
  netSupervisor();

  if ((int32_t)(millis() - nextHeartbeat) >= 0) {
    nextHeartbeat = millis() + HEARTBEAT_PERIOD_MS;
    Serial.printf("[hb] up %lus heap %u/%u wifi %d rssi %d level %.1fdBFS",
                  (unsigned long)(millis() / 1000),
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(),
                  (int)WiFi.status(), (int)WiFi.RSSI(), lastLevelDb);
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

#if ENABLE_MIC
  if (i2sOk) {
    size_t bytesRead = 0;
    // 512 samples at 16 kHz blocks for ~32 ms. A finite timeout keeps the
    // supervisor and ardb->update() running if the mic stops clocking, where
    // portMAX_DELAY would wedge the loop forever.
    const esp_err_t result = i2s_read(I2S_PORT, sampleBuffer,
                                      sizeof(sampleBuffer), &bytesRead,
                                      pdMS_TO_TICKS(100));

    if (result == ESP_OK && bytesRead == sizeof(sampleBuffer)) {
      // --- 1. Normalise and remove the DC offset -------------------------
      // I2S MEMS mics carry a large DC bias. Windowing a biased signal smears
      // it across the low bins, which is why the original had to discard bins
      // 0 and 1. Subtracting the block mean fixes it at the source.
      // The mean is accumulated in int64 over the raw counts, which is exact.
      // A float accumulator would lose precision against a large DC bias,
      // which is the very thing being measured.
      int64_t rawSum = 0;
      for (uint16_t i = 0; i < samples; i++) {
        rawSum += (sampleBuffer[i] >> 8);  // 24-bit left-justified
      }
      const float meanCounts = (float)rawSum / (float)samples;

      // Subtract in counts, then normalise to +/-1.0 full scale.
      float sumSq = 0.0f;
      for (uint16_t i = 0; i < samples; i++) {
        const float centred =
            ((float)(sampleBuffer[i] >> 8) - meanCounts) * kInvFullScale24;
        vReal[i] = centred;
        vImag[i] = 0.0f;
        sumSq += centred * centred;
      }
      lastLevelDb = amplitudeToDb(sqrtf(sumSq / samples));

      // --- 2. FFT --------------------------------------------------------
      // Hann reads better than Hamming for a display spectrum: lower
      // sidelobes, so adjacent bands stay distinguishable.
      FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();  // magnitudes now in vReal[0..samples/2]

      // --- 3. Magnitude -> calibrated amplitude --------------------------
      // For a tone of amplitude A windowed by Hann (coherent gain 0.5), the
      // peak bin magnitude is A*N*0.5/2, so A = 4*mag/N. That makes a
      // full-scale sine read 0 dBFS.
      const float ampScale = 4.0f / (float)samples;
      for (uint16_t b = 0; b < samples / 2; b++) {
        amp[b] = vReal[b] * ampScale;
      }

      // --- 4. Fold bins into the 64 bands --------------------------------
      for (uint8_t i = 0; i < kBandCount; i++) {
        const float db =
            amplitudeToDb(bandAmplitude(amp, bandBinLo[i], bandBinHi[i]));
        const float coeff = (db > bandDb[i]) ? kAttack : kDecay;
        bandDb[i] += coeff * (db - bandDb[i]);
        spectrumPayload[i] = dbToByte(bandDb[i]);
      }

      // --- 5. Lumped bass / mid / treble ---------------------------------
      const uint16_t bassLo = 2;                           //   62.5 Hz
      const uint16_t bassHi = (uint16_t)(250.0f / binHz);  //  250 Hz
      const uint16_t midHi = (uint16_t)(2000.0f / binHz);  // 2000 Hz
      const uint16_t trebleHi = samples / 2;               // 8000 Hz
      lastBass = amplitudeToDb(bandAmplitude(amp, bassLo, bassHi));
      lastMid = amplitudeToDb(bandAmplitude(amp, bassHi, midHi));
      lastTreble = amplitudeToDb(bandAmplitude(amp, midHi, trebleHi));

      // --- 6. Peak frequency with parabolic interpolation ----------------
      uint16_t peakBin = 2;
      for (uint16_t b = 2; b < samples / 2; b++) {
        if (amp[b] > amp[peakBin]) peakBin = b;
      }
      if (amplitudeToDb(amp[peakBin]) <= kFloorDb) {
        lastPeakHz = 0.0f;  // nothing above the floor
      } else if (peakBin > 0 && peakBin + 1 < samples / 2) {
        const float a = amp[peakBin - 1];
        const float b = amp[peakBin];
        const float c = amp[peakBin + 1];
        const float denom = a - 2.0f * b + c;
        const float delta =
            (fabsf(denom) < 1e-12f) ? 0.0f : 0.5f * (a - c) / denom;
        lastPeakHz = ((float)peakBin + delta) * binHz;
      } else {
        lastPeakHz = (float)peakBin * binHz;
      }

      // --- 7. Publish ----------------------------------------------------
#if ENABLE_ARDB
      if (ardbReady && ardb && ardb->connected()) {
        // The typed array overload infers sizeof(spectrumPayload) == 64.
        const bool anySent = ardb->print(tSpectrum, spectrumPayload);

        // Feeds netSupervisor(). Rate-limited calls return false, so this
        // only advances when bytes really reached the broker.
        if (anySent) g_lastPublishOk = millis();
      }
#endif
    } else if (result != ESP_OK && result != ESP_ERR_TIMEOUT) {
      Serial.printf("[i2s] read error 0x%x\n", result);
    }
  } else {
    delay(20);
  }
#else
  delay(20);
#endif

  if ((int32_t)(millis() - nextPlotMs) >= 0) {
    nextPlotMs = millis() + SERIAL_PLOT_PERIOD_MS;
    // Serial Plotter view of the lumped bands plus the peak.
    Serial.printf("Bass:%.1f Mid:%.1f Treble:%.1f Level:%.1f PeakHz:%.0f\n",
                  lastBass, lastMid, lastTreble, lastLevelDb, lastPeakHz);
  }
}
