// ESP32 serial <-> ESP-NOW bridge.
//
// Serial in:  one packet per line, uppercase or lowercase hex, '\n' terminated.
//             The bytes are broadcast over ESP-NOW.
// Serial out: every received ESP-NOW packet is printed as one uppercase hex line.
//             Lines starting with '#' are diagnostics; the Python side ignores them.
//
// Demo aids:
//   BOOT button toggles "radio mute": all TX and RX are dropped, so link loss can be
//   shown on demand. The LED stays solid while muted.
//   The LED flickers on every packet sent or received.
//
// The bridge knows nothing about the protocol. Seq, ACK, retry and buffering all
// live in protocol/ (Python), so swapping ESP-NOW for LoRa only touches this file.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const int BAUD = 115200;
const int WIFI_CHANNEL = 1;  // both boards must match; try 6 or 11 if the venue is noisy
const int MAX_LEN = ESP_NOW_MAX_DATA_LEN;  // 250 bytes
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Pins differ per board, so platformio.ini sets them for each env.
const int BUTTON_PIN = MUTE_BUTTON_PIN;  // active low (pull-up)
const int LED_PIN = ACTIVITY_LED_PIN;
const unsigned long FLASH_MS = 30;
const unsigned long DEBOUNCE_MS = 50;

struct Packet {
  int len;
  uint8_t data[MAX_LEN];
};

// The receive callback runs on the WiFi task, where printing to Serial is unsafe.
// It just copies the packet into this queue; loop() does the printing.
QueueHandle_t rxQueue;

char lineBuf[2 * MAX_LEN + 1];
int lineLen = 0;
bool overflow = false;  // true while discarding the rest of a too-long line

volatile bool muted = false;  // read by the WiFi-task callback, so volatile
unsigned long ledOffAt = 0;
bool lastButton = HIGH;
unsigned long buttonChangedAt = 0;

void onReceive(const uint8_t *data, int len) {
  if (muted || len <= 0 || len > MAX_LEN) return;
  Packet p;
  p.len = len;
  memcpy(p.data, data, len);
  xQueueSend(rxQueue, &p, 0);  // drop if full rather than block the WiFi task
}

// The callback signature changed in Arduino-ESP32 core 3.x.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceiveCb(const esp_now_recv_info_t *info, const uint8_t *data, int len) { onReceive(data, len); }
#else
void onReceiveCb(const uint8_t *mac, const uint8_t *data, int len) { onReceive(data, len); }
#endif

void flashLed() {
  if (muted) return;
  digitalWrite(LED_PIN, HIGH);
  ledOffAt = millis() + FLASH_MS;
}

void updateLed() {
  if (!muted && ledOffAt && millis() >= ledOffAt) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }
}

// Toggle mute on each press (falling edge), with a simple debounce.
void checkButton() {
  bool now = digitalRead(BUTTON_PIN);
  if (now == lastButton || millis() - buttonChangedAt < DEBOUNCE_MS) return;
  lastButton = now;
  buttonChangedAt = millis();
  if (now == LOW) {
    muted = !muted;
    digitalWrite(LED_PIN, muted ? HIGH : LOW);
    Serial.println(muted ? "# muted" : "# unmuted");
  }
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Decode the hex line and broadcast it. Bad input gets a '#' diagnostic, never a crash.
void sendLine(const char *hex, int hexLen) {
  if (hexLen == 0 || muted) return;
  if (hexLen % 2 != 0 || hexLen / 2 > MAX_LEN) {
    Serial.println("# err bad length");
    return;
  }
  uint8_t out[MAX_LEN];
  for (int i = 0; i < hexLen / 2; i++) {
    int hi = hexValue(hex[2 * i]);
    int lo = hexValue(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      Serial.println("# err bad hex");
      return;
    }
    out[i] = (hi << 4) | lo;
  }
  if (esp_now_send(BROADCAST, out, hexLen / 2) != ESP_OK) {
    Serial.println("# err send failed");
    return;
  }
  flashLed();
}

void printPacket(const Packet &p) {
  static const char digits[] = "0123456789ABCDEF";
  for (int i = 0; i < p.len; i++) {
    Serial.write(digits[p.data[i] >> 4]);
    Serial.write(digits[p.data[i] & 0x0F]);
  }
  Serial.write('\n');
}

void setup() {
  Serial.begin(BAUD);
  // GPIO34-39 are input-only with no internal pull-up (the Feather V2 has one on the board).
  pinMode(BUTTON_PIN, BUTTON_PIN >= 34 ? INPUT : INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  rxQueue = xQueueCreate(16, sizeof(Packet));

  // ESP-NOW needs the radio in station mode, but we never join a network.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# err esp_now_init failed");
    return;
  }
  esp_now_register_recv_cb(onReceiveCb);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.print("# bridge ready mac=");
  Serial.print(WiFi.macAddress());
  Serial.print(" channel=");
  Serial.println(WIFI_CHANNEL);
}

void loop() {
  checkButton();
  updateLed();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflow) Serial.println("# err line too long");
      else sendLine(lineBuf, lineLen);
      lineLen = 0;
      overflow = false;
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      overflow = true;  // keep reading until newline, then drop the whole line
    }
  }

  Packet p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    printPacket(p);
    flashLed();
  }
}
