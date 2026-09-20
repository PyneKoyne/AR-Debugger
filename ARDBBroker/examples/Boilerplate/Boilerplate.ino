// This is the maximum number of runtime-discovered streams. It reserves all
// registry storage at compile time; adjust it before including ARDBBroker.h.
#define ARDB_HEAD_MAX_STREAMS 8
#include <ARDBBroker.h>

namespace {

// Copy this example to a private sketch before replacing its placeholders.
// All deployment-specific values belong in that sketch, not library source.
constexpr ardb_network::Config kNetwork = {
    ardb_network::Mode::AccessPoint,  // Change to Mode::Station for existing Wi-Fi.
    "replace-with-wifi-name",
    "replace-with-wifi-password",
    ardb_network::kDefaultAccessPointAddress,
};

// Paste a certificate whose IP SAN/DNS SAN matches the host Quest will fetch.
// For the default AP, that host is 192.168.4.1.
constexpr char kCertificatePem[] = R"ARDB_CERT(
-----BEGIN CERTIFICATE-----
replace-with-your-development-certificate
-----END CERTIFICATE-----
)ARDB_CERT";

constexpr char kPrivateKeyPem[] = R"ARDB_KEY(
-----BEGIN PRIVATE KEY-----
replace-with-your-development-private-key
-----END PRIVATE KEY-----
)ARDB_KEY";

ardb_broker::Runtime ardb(kNetwork, kCertificatePem, kPrivateKeyPem);

void waitForSerial(uint32_t timeoutMs = 3000) {
  const uint32_t startedAtMs = millis();
  while (!Serial && millis() - startedAtMs < timeoutMs) {
    delay(10);
  }
}

void setActivityLed(bool on) {
#if defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

void initialiseActivityLed() {
#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  setActivityLed(false);
}

[[noreturn]] void haltAfterNetworkFailure() {
  while (true) {
#if defined(LED_BUILTIN)
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
    delay(250);
  }
}

void printServiceAddresses() {
  const IPAddress address = ardb.address();
  Serial.print("Network mode: ");
  Serial.println(ardb.networkModeName());
  Serial.print("Broker ready: mqtt://");
  Serial.print(address);
  Serial.print(':');
  Serial.println(ardb_network::kMqttPort);
  Serial.print("Head API ready: https://");
  Serial.print(address);
  Serial.println(ardb_head::kHealthPath);
}

}  // namespace

void setup() {
  initialiseActivityLed();

  Serial.begin(115200);
  waitForSerial();
  Serial.println();
  Serial.println("ARDB broker and head node");

  if (!ardb.begin()) {
    haltAfterNetworkFailure();
  }

  setActivityLed(true);
  printServiceAddresses();
}

void loop() {
  ardb.update();
}
