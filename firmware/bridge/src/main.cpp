// ESP32 serial <-> radio bridge (ESP-NOW, plus LoRa 915 MHz on Heltec V3).
//
// Serial in:  one packet per line, uppercase or lowercase hex, '\n' terminated.
//             The bytes are broadcast over every enabled radio.
// Serial out: every packet received on any radio is printed as one uppercase hex line.
//             Lines starting with '#' are diagnostics; the Python side ignores them.
//
// Dual path (HAS_LORA): each packet goes out on ESP-NOW *and* LoRa. The receiver may get
// two copies; the Python link layer's dedup drops the second, exactly like a retry.
//
// Demo aids:
//   The button (BOOT / PRG) cycles the radio mode:
//     with LoRa:    BOTH -> LORA ONLY (ESP-NOW off) -> MUTED -> BOTH
//     without LoRa: BOTH -> MUTED -> BOTH
//   MUTED drops all TX and RX, so link loss can be shown on demand. The LED stays solid
//   while muted and flickers on every packet otherwise.
//   HAS_OLED: the screen shows DRONE/GROUND, the mode, per-radio signal bars, RSSI and
//   packet counts, and the last packet.
//
// The bridge knows nothing about the protocol. Seq, ACK, retry and buffering all
// live in protocol/ (Python). The OLED peeks at the type and seq bytes only to label things.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#ifdef HAS_LORA
#include <RadioLib.h>
#endif
#ifdef HAS_OLED
#include <U8g2lib.h>
#endif

const int BAUD = 115200;
const int WIFI_CHANNEL = 1;  // both boards must match; try 6 or 11 if the venue is noisy
const int MAX_LEN = ESP_NOW_MAX_DATA_LEN;  // 250 bytes (LoRa allows 255, so this fits both)
const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Pins differ per board, so platformio.ini sets them for each env.
const int BUTTON_PIN = MUTE_BUTTON_PIN;  // active low (pull-up)
const int LED_PIN = ACTIVITY_LED_PIN;
const unsigned long FLASH_MS = 30;
const unsigned long DEBOUNCE_MS = 50;

enum Mode { MODE_BOTH, MODE_LORA_ONLY, MODE_MUTED };
enum Path { PATH_ESPNOW, PATH_LORA };

struct Packet {
  int len;
  uint8_t path;
  uint8_t data[MAX_LEN];
};

// The ESP-NOW receive callback runs on the WiFi task, where printing to Serial is unsafe.
// It just copies the packet into this queue; loop() does the printing.
QueueHandle_t rxQueue;

char lineBuf[2 * MAX_LEN + 1];
int lineLen = 0;
bool overflow = false;  // true while discarding the rest of a too-long line

volatile Mode mode = MODE_BOTH;  // read by the WiFi-task callback, so volatile
unsigned long ledOffAt = 0;
bool lastButton = HIGH;
unsigned long buttonChangedAt = 0;

// Stats for the OLED only (index by Path).
unsigned long txCount[2] = {0, 0};
volatile unsigned long rxCount[2] = {0, 0};
volatile int rssi[2] = {0, 0};
unsigned long lastRxAt[2] = {0, 0};
float loraSnr = 0;
uint8_t lastType = 0;
unsigned int lastSeq = 0;
bool lastWasTx = false;

// Which side this board is on, for the OLED label only. Learned from traffic: the drone
// sends DETECTIONs and receives COMMANDs; the ground does the opposite.
enum Role { ROLE_UNKNOWN, ROLE_DRONE, ROLE_GROUND };
Role role = ROLE_UNKNOWN;
const uint8_t MAGIC = 0xD2, TYPE_DETECTION = 0x01, TYPE_COMMAND = 0x04;

void notePacket(const uint8_t *data, int len, bool tx) {
  if (len < 4 || data[0] != MAGIC) return;
  lastType = data[1];
  lastSeq = data[2] | (data[3] << 8);
  lastWasTx = tx;
  if (data[1] == TYPE_DETECTION) role = tx ? ROLE_DRONE : ROLE_GROUND;
  if (data[1] == TYPE_COMMAND) role = tx ? ROLE_GROUND : ROLE_DRONE;
}

bool espNowOn() { return mode == MODE_BOTH; }
bool loraOn() { return mode != MODE_MUTED; }

// ---------------------------------------------------------------- LoRa (Heltec V3)
#ifdef HAS_LORA
// SX1262 wiring on the Heltec WiFi LoRa 32 V3: NSS 8, DIO1 14, RST 12, BUSY 13 (SPI 9/11/10).
SX1262 radio = new Module(8, 14, 12, 13);
const float LORA_MHZ = 915.0;  // Canada/US ISM band. The board is the 863-928 MHz variant.
const float LORA_BW = 125.0;   // kHz
const int LORA_SF = 7;         // lowest airtime: ~57 ms for a 21-byte DETECTION
const int LORA_CR = 5;         // 4/5
const int LORA_DBM = 10;       // plenty indoors; max is 22
bool loraReady = false;
volatile bool loraIrq = false;  // DIO1: TX done or RX done
bool loraTxBusy = false;

// LoRa is half duplex and one packet takes ~50 ms, so outgoing packets wait here.
const int LORA_TXQ = 8;
Packet loraTxQueue[LORA_TXQ];
int loraTxHead = 0, loraTxCount = 0;

void IRAM_ATTR onLoraIrq() { loraIrq = true; }

void loraSetup() {
  SPI.begin(9, 11, 10, 8);
  // TCXO at 1.8 V and DIO2 as the RF switch, as on Heltec's own V3 configuration.
  int state = radio.begin(LORA_MHZ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                          LORA_DBM, 8, 1.8);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("# err lora begin ");
    Serial.println(state);
    return;
  }
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);
  radio.setDio1Action(onLoraIrq);
  radio.startReceive();
  loraReady = true;
}

void loraQueue(const uint8_t *data, int len) {
  if (!loraReady || loraTxCount == LORA_TXQ) return;  // full: ESP-NOW and retries cover it
  Packet &slot = loraTxQueue[(loraTxHead + loraTxCount) % LORA_TXQ];
  slot.len = len;
  memcpy(slot.data, data, len);
  loraTxCount++;
}

void loraPoll() {
  if (!loraReady) return;
  if (loraIrq) {
    loraIrq = false;
    if (loraTxBusy) {
      radio.finishTransmit();
      loraTxBusy = false;
    } else {
      Packet p;
      p.path = PATH_LORA;
      p.len = radio.getPacketLength();
      bool ok = p.len > 0 && p.len <= MAX_LEN && radio.readData(p.data, p.len) == RADIOLIB_ERR_NONE;
      if (ok && loraOn()) {
        rssi[PATH_LORA] = (int)radio.getRSSI();
        loraSnr = radio.getSNR();
        xQueueSend(rxQueue, &p, 0);
      }
    }
    radio.startReceive();
  }
  if (!loraTxBusy && loraTxCount > 0) {
    Packet &next = loraTxQueue[loraTxHead];
    if (radio.startTransmit(next.data, next.len) == RADIOLIB_ERR_NONE) loraTxBusy = true;
    loraTxHead = (loraTxHead + 1) % LORA_TXQ;
    loraTxCount--;
  }
}
#endif

// ---------------------------------------------------------------- OLED (Heltec V3)
#ifdef HAS_OLED
// SSD1306 128x64 on I2C SDA 17 / SCL 18, reset 21. Vext (GPIO36, active low) powers it.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, /*reset=*/21, /*clock=*/18, /*data=*/17);
unsigned long oledAt = 0;

const char *typeName(uint8_t t) {
  switch (t) {
    case 0x01: return "DETECTION";
    case 0x02: return "ACK";
    case 0x03: return "HEARTBEAT";
    case 0x04: return "COMMAND";
    default: return "?";
  }
}

void oledSetup() {
  pinMode(36, OUTPUT);
  digitalWrite(36, LOW);
  delay(50);
  oled.begin();
  oled.clearBuffer();
  oled.setFont(u8g2_font_helvB12_tr);
  oled.drawStr(24, 30, "ArcDrone");
  oled.setFont(u8g2_font_5x7_tr);
  oled.drawStr(22, 46, "ESP-NOW + LoRa 915");
  oled.sendBuffer();
}

// Phone-style signal bars from RSSI. Empty outlines = no recent packets.
void drawBars(int x, int y, int bars) {  // y = bottom
  for (int i = 0; i < 4; i++) {
    int h = 3 + 3 * i;
    if (i < bars) oled.drawBox(x + 4 * i, y - h, 3, h);
    else oled.drawFrame(x + 4 * i, y - h, 3, h);
  }
}

int barsFor(int path, int dbm) {
  const int *t;
  static const int espnow[] = {-55, -67, -78, -90};
  static const int lora[] = {-70, -90, -105, -115};
  t = path == PATH_ESPNOW ? espnow : lora;
  int bars = 0;
  for (int i = 3; i >= 0; i--) if (dbm > t[i]) bars = 4 - i;
  return bars;
}

void radioRow(int y, const char *name, int path, bool on) {  // 20 px tall, starting at y
  char buf[24];
  unsigned long age = lastRxAt[path] ? millis() - lastRxAt[path] : ULONG_MAX;
  bool live = on && age < 3000;
  if (!on) {
    drawBars(0, y + 13, 0);
    oled.drawLine(0, y + 1, 14, y + 13);  // crossed out
  } else {
    drawBars(0, y + 13, live ? barsFor(path, rssi[path]) : 0);
  }
  oled.setFont(u8g2_font_6x10_tr);
  oled.drawStr(19, y + 9, name);
  if (on && age < 250) oled.drawDisc(19 + oled.getStrWidth(name) + 5, y + 5, 2);  // activity blip
  oled.setFont(u8g2_font_5x7_tr);
  if (!on) snprintf(buf, sizeof buf, "OFF");
  else if (live) snprintf(buf, sizeof buf, "%d dBm", rssi[path]);
  else snprintf(buf, sizeof buf, "no signal");
  oled.drawStr(127 - oled.getStrWidth(buf), y + 9, buf);
  snprintf(buf, sizeof buf, "tx %lu   rx %lu", txCount[path], (unsigned long)rxCount[path]);
  oled.drawStr(19, y + 18, buf);
}

void oledDraw() {
  if (millis() - oledAt < 200) return;  // 5 Hz: a full redraw blocks the loop for ~25 ms of I2C
  oledAt = millis();
  char buf[32];
  const char *who = role == ROLE_DRONE ? "DRONE" : role == ROLE_GROUND ? "GROUND" : "BRIDGE";
  oled.clearBuffer();

  // Header: which side this board is, and the radio mode as a pill.
  oled.setFont(u8g2_font_helvB10_tr);
  oled.drawStr(0, 11, who);
  const char *m = mode == MODE_MUTED ? "MUTED" : mode == MODE_LORA_ONLY ? "LORA" : "DUAL";
  oled.setFont(u8g2_font_5x7_tr);
  int w = oled.getStrWidth(m) + 8;
  oled.drawRBox(127 - w, 1, w, 11, 3);
  oled.setDrawColor(0);
  oled.drawStr(127 - w + 4, 9, m);
  oled.setDrawColor(1);
  oled.drawHLine(0, 14, 128);

  if (mode == MODE_MUTED) {
    oled.drawRBox(4, 20, 120, 30, 4);
    oled.setDrawColor(0);
    oled.setFont(u8g2_font_helvB10_tr);
    oled.drawStr(64 - oled.getStrWidth("RADIO MUTED") / 2, 40, "RADIO MUTED");
    oled.setDrawColor(1);
    oled.setFont(u8g2_font_5x7_tr);
    oled.drawStr(64 - oled.getStrWidth("press PRG to restore") / 2, 61, "press PRG to restore");
    oled.sendBuffer();
    return;
  }

  radioRow(17, "ESP-NOW", PATH_ESPNOW, espNowOn());
#ifdef HAS_LORA
  radioRow(38, "LoRa 915", PATH_LORA, loraOn() && loraReady);
#endif

  // Footer: the last packet, with an arrow for its direction.
  if (lastType) {
    oled.setFont(u8g2_font_5x7_tr);
    if (lastWasTx) oled.drawTriangle(0, 58, 0, 64, 4, 61);   // > out
    else oled.drawTriangle(4, 58, 4, 64, 0, 61);             // < in
    snprintf(buf, sizeof buf, "%s %s #%u", lastWasTx ? "TX" : "RX", typeName(lastType), lastSeq);
    oled.drawStr(8, 64, buf);
  }
  oled.sendBuffer();
}
#endif

// ---------------------------------------------------------------- ESP-NOW + serial

void onReceive(const uint8_t *data, int len, int rssiDbm) {
  if (!espNowOn() || len <= 0 || len > MAX_LEN) return;
  Packet p;
  p.len = len;
  p.path = PATH_ESPNOW;
  memcpy(p.data, data, len);
  rssi[PATH_ESPNOW] = rssiDbm;
  xQueueSend(rxQueue, &p, 0);  // drop if full rather than block the WiFi task
}

// The callback signature changed in Arduino-ESP32 core 3.x (which also gives us RSSI).
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceiveCb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  onReceive(data, len, info->rx_ctrl ? info->rx_ctrl->rssi : 0);
}
#else
void onReceiveCb(const uint8_t *mac, const uint8_t *data, int len) { onReceive(data, len, 0); }
#endif

void flashLed() {
  if (mode == MODE_MUTED) return;
  digitalWrite(LED_PIN, HIGH);
  ledOffAt = millis() + FLASH_MS;
}

void updateLed() {
  if (mode != MODE_MUTED && ledOffAt && millis() >= ledOffAt) {
    digitalWrite(LED_PIN, LOW);
    ledOffAt = 0;
  }
}

// Advance the radio mode on each press (falling edge), with a simple debounce.
void checkButton() {
  bool now = digitalRead(BUTTON_PIN);
  if (now == lastButton || millis() - buttonChangedAt < DEBOUNCE_MS) return;
  lastButton = now;
  buttonChangedAt = millis();
  if (now != LOW) return;
#ifdef HAS_LORA
  mode = mode == MODE_BOTH ? MODE_LORA_ONLY : mode == MODE_LORA_ONLY ? MODE_MUTED : MODE_BOTH;
#else
  mode = mode == MODE_MUTED ? MODE_BOTH : MODE_MUTED;
#endif
  digitalWrite(LED_PIN, mode == MODE_MUTED ? HIGH : LOW);
  Serial.println(mode == MODE_MUTED ? "# muted" : mode == MODE_LORA_ONLY ? "# lora only" : "# unmuted");
}

int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Decode the hex line and broadcast it. Bad input gets a '#' diagnostic, never a crash.
void sendLine(const char *hex, int hexLen) {
  if (hexLen == 0 || mode == MODE_MUTED) return;
  if (hexLen % 2 != 0 || hexLen / 2 > MAX_LEN) {
    Serial.println("# err bad length");
    return;
  }
  uint8_t out[MAX_LEN];
  int len = hexLen / 2;
  for (int i = 0; i < len; i++) {
    int hi = hexValue(hex[2 * i]);
    int lo = hexValue(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) {
      Serial.println("# err bad hex");
      return;
    }
    out[i] = (hi << 4) | lo;
  }
  if (espNowOn()) {
    if (esp_now_send(BROADCAST, out, len) != ESP_OK) Serial.println("# err send failed");
    else txCount[PATH_ESPNOW]++;
  }
#ifdef HAS_LORA
  if (loraOn() && loraReady) {
    loraQueue(out, len);
    txCount[PATH_LORA]++;
  }
#endif
  notePacket(out, len, true);
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
  pinMode(BUTTON_PIN, BUTTON_PIN >= 34 && BUTTON_PIN <= 39 ? INPUT : INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  rxQueue = xQueueCreate(16, sizeof(Packet));

#ifdef HAS_OLED
  oledSetup();
#endif

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

#ifdef HAS_LORA
  loraSetup();
#endif

  Serial.print("# bridge ready mac=");
  Serial.print(WiFi.macAddress());
  Serial.print(" channel=");
  Serial.print(WIFI_CHANNEL);
#ifdef HAS_LORA
  Serial.print(loraReady ? " lora=915MHz/SF7" : " lora=FAILED");
#endif
  Serial.println();
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

#ifdef HAS_LORA
  loraPoll();
#endif

  Packet p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) {
    printPacket(p);
    rxCount[p.path]++;
    lastRxAt[p.path] = millis();
    notePacket(p.data, p.len, false);
    flashLed();
  }

#ifdef HAS_OLED
  oledDraw();
#endif
}
