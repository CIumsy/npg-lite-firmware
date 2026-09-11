// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2026 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2026 Upside Down Labs - contact@upsidedownlabs.tech

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// ESP32 based IR Controller using an Adafruit IR Transceiver on two GPIO lines
// Hold the user button to record an IR signal, short press to fire the active one.
// Names and the active slot live in NVS. Captured waveforms live in LittleFS.

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <Adafruit_NeoPixel.h>

// Pins
#define IR_RECV_PIN     23
#define IR_SEND_PIN     22
#define USER_BTN_PIN    9

// NeoPixel status ring. Pixel 0 BLE, pixel 5 battery.
#define PIN_NEOPIXEL        15
#define PIXEL_COUNT         6
#define BLE_LED             0
#define BATTERY_LED         5
#define BLUE_LED_DURATION   100
#define BATTERY_VOLTAGE_PIN A6

// Timing
#define HOLD_THRESHOLD  1500
#define DEBOUNCE_MS     200
#define LIST_DELAY_MS   1000
#define LIST_PACE_MS    15     // one list entry per connection interval
#define RECORD_TIMEOUT  15000

// Limits
#define MAX_COMMANDS    50
#define MAX_NAME_LEN    16
#define RAW_BUF_LEN     1024
#define IR_TIMEOUT_MS   15
#define IR_FREQ_HZ      38000

#define NVS_NAMESPACE   "ir"
#define BLE_NAME        "NPG-IR"
#define SVC_UUID        "12345678-1234-1234-1234-1234567890ab"
#define NOTIFY_UUID     "12345678-1234-1234-1234-1234567890ac"
#define WRITE_UUID      "12345678-1234-1234-1234-1234567890ad"

// Board -> app
#define EV_LIST_ENTRY   0x12   // [id, active, nameLen, name...]
#define EV_LIST_END     0x14   
#define EV_CAPTURE      0x20   // [isDup, dupId, proto(2), bits(2), value(8)]
#define EV_SAVED        0x21   // [id, nameLen, name...]
#define EV_DELETED      0x22   // [id]
#define EV_ACTIVE       0x23   // [id]
#define EV_OK           0x24   
#define EV_FAIL         0x25   
#define EV_WIPED        0x26   
#define EV_LISTENING    0x27   // [timeoutSec]
#define EV_LISTEN_END   0x28

// App -> board
#define CMD_SET_ACTIVE  0x01   // [id]
#define CMD_DELETE      0x02   // [id]
#define CMD_RENAME      0x03   // [id, name...]
#define CMD_SAVE_NEW    0x04   // [name...]
#define CMD_SAVE_OVER   0x05   // [id, name...]
#define CMD_GET_LIST    0x06   
#define CMD_FIRE        0x07   // [id]
#define CMD_WIPE        0x08   
#define CMD_CANCEL_REC  0x09
#define CMD_DISCARD     0x0A

IRrecv irrecv(IR_RECV_PIN, RAW_BUF_LEN, IR_TIMEOUT_MS, true);
IRsend irsend(IR_SEND_PIN);
Preferences prefs;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// NeoPixel state
uint32_t bleColor     = 0;
uint32_t batteryColor = 0;

uint32_t shownBleColor     = 0xFFFFFFFF;
uint32_t shownBatteryColor = 0xFFFFFFFF;

unsigned long lastCmdSentMs = 0;

// ---- Battery ----
#define BATTERY_CHECK_MS   30000   // read the battery every 30 s
#define BATTERY_SAMPLE_MS  100     // accumulate a reading this often

unsigned long lastBatteryCheck = 0;
unsigned long lastBatterySample = 0;
uint32_t batteryWinSum = 0;
uint16_t batteryWinCount = 0;
int lastBatteryPct = -1;
uint8_t risingCount = 0;
const uint8_t RISING_THRESHOLD = 3;
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};
const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};
const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

struct CmdEntry {
  bool exists;
  char name[MAX_NAME_LEN + 1];
};

CmdEntry cmds[MAX_COMMANDS];
int activeId = -1;

// Capture held in RAM until the app supplies a name
bool pendingValid = false;
decode_results pendingResult;
volatile uint16_t pendingRaw[RAW_BUF_LEN];

// Shared by loadIR callers. Only ever touched from loop().
volatile uint16_t scratchRaw[RAW_BUF_LEN];

// Recording latches on after a long press and stays on after the button is
// released, until a signal arrives, the timeout expires, or it is cancelled.
bool     recording = false;
uint32_t recordAt  = 0;

// BLE
BLECharacteristic* pNotify = nullptr;
bool bleConnected = false;
uint32_t connectedAt = 0;

// Work deferred out of the BLE callback so IR and filesystem access
// stay on the main task, which has a much larger stack.
// Command list streaming. -1 means idle, otherwise the next slot to look at.
int      listCursor   = -1;
uint32_t lastListSend = 0;

volatile bool reqList   = false;
volatile bool reqWipe   = false;
volatile bool reqCancel = false;
volatile int  reqFire   = -1;

// Sends one notification and returns. The stack queues a handful of packets,
// so isolated events are safe to fire back to back. Only a long run needs
// pacing, and the command list is the one place that happens. See serviceList.
void bleNotify(const uint8_t* buf, size_t len) {
  if (!bleConnected || !pNotify) return;
  pNotify->setValue((uint8_t*)buf, len);
  pNotify->notify();
}

void bleNotify1(uint8_t op) { bleNotify(&op, 1); }

void nvsLoad() {
  prefs.begin(NVS_NAMESPACE, true);
  activeId = prefs.getInt("act", -1);
  for (int i = 0; i < MAX_COMMANDS; i++) {
    char key[8];
    snprintf(key, sizeof(key), "n%d", i);
    cmds[i].exists = prefs.isKey(key);
    if (cmds[i].exists) {
      String s = prefs.getString(key, "");
      strncpy(cmds[i].name, s.c_str(), MAX_NAME_LEN);
      cmds[i].name[MAX_NAME_LEN] = '\0';
    } else {
      cmds[i].name[0] = '\0';
    }
  }
  prefs.end();
  if (activeId >= 0 && (activeId >= MAX_COMMANDS || !cmds[activeId].exists)) activeId = -1;
}

void nvsSave() {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putInt("act", activeId);
  for (int i = 0; i < MAX_COMMANDS; i++) {
    char key[8];
    snprintf(key, sizeof(key), "n%d", i);
    if (cmds[i].exists) prefs.putString(key, cmds[i].name);
    else                prefs.remove(key);
  }
  prefs.end();
}

String irPath(int id) { return String("/ir") + id; }

// Layout: proto(2) bits(2) value(8) rawlen(2) state(kStateSizeMax) rawbuf(rawlen*2)
const size_t IR_HEADER_LEN = 2 + 2 + 8 + 2 + kStateSizeMax;

bool saveIR(int id, decode_results* res) {
  File f = LittleFS.open(irPath(id), "w");
  if (!f) return false;
  uint16_t proto  = (uint16_t)res->decode_type;
  uint16_t bits   = res->bits;
  uint16_t rawlen = res->rawlen;
  uint64_t value  = res->value;
  // count what the writes accept. f.size() on a file that is still open
  // does not include buffered data, so it cannot be used to verify this.
  size_t written = 0;
  written += f.write((uint8_t*)&proto,  2);
  written += f.write((uint8_t*)&bits,   2);
  written += f.write((uint8_t*)&value,  8);
  written += f.write((uint8_t*)&rawlen, 2);
  written += f.write((uint8_t*)res->state, kStateSizeMax);
  for (uint16_t i = 0; i < rawlen; i++) {
    uint16_t v = res->rawbuf[i];
    written += f.write((uint8_t*)&v, 2);
  }
  f.close();

  size_t expected = IR_HEADER_LEN + (size_t)rawlen * 2;
  if (written != expected) {
    Serial.printf("[FS] slot %d wrote %u of %u bytes\n",
                  id, (unsigned)written, (unsigned)expected);
    LittleFS.remove(irPath(id));
    return false;
  }
  return true;
}

// res->rawbuf must point at a buffer of RAW_BUF_LEN entries.
bool loadIR(int id, decode_results* res) {
  File f = LittleFS.open(irPath(id), "r");
  if (!f) return false;
  if (f.size() < IR_HEADER_LEN) { f.close(); return false; }

  uint16_t proto, bits, rawlen;
  uint64_t value;
  f.read((uint8_t*)&proto,  2);
  f.read((uint8_t*)&bits,   2);
  f.read((uint8_t*)&value,  8);
  f.read((uint8_t*)&rawlen, 2);

  // Never trust a length read back from flash
  if (rawlen > RAW_BUF_LEN || f.size() < IR_HEADER_LEN + (size_t)rawlen * 2) {
    f.close();
    return false;
  }

  f.read((uint8_t*)res->state, kStateSizeMax);
  for (uint16_t i = 0; i < rawlen; i++) {
    uint16_t v = 0;
    f.read((uint8_t*)&v, 2);
    res->rawbuf[i] = v;
  }
  f.close();

  res->decode_type = (decode_type_t)proto;
  res->bits   = bits;
  res->value  = value;
  res->rawlen = rawlen;
  return true;
}

void deleteIR(int id) {
  String path = irPath(id);
  if (LittleFS.exists(path)) LittleFS.remove(path);
}

int freeSlot() {
  for (int i = 0; i < MAX_COMMANDS; i++)
    if (!cmds[i].exists) return i;
  return -1;
}

int findDuplicate(decode_results* res) {
  if (res->decode_type == UNKNOWN) return -1;
  decode_results stored;
  stored.rawbuf = scratchRaw;
  for (int i = 0; i < MAX_COMMANDS; i++) {
    if (!cmds[i].exists) continue;
    if (!loadIR(i, &stored)) continue;
    if (stored.decode_type == res->decode_type && stored.value == res->value) return i;
  }
  return -1;
}

bool fireCommand(int id) {
  if (id < 0 || id >= MAX_COMMANDS || !cmds[id].exists) return false;

  decode_results res;
  res.rawbuf = scratchRaw;
  if (!loadIR(id, &res)) return false;

  Serial.printf("[IR] fire %d proto=%s bits=%d\n",
                id, typeToString(res.decode_type).c_str(), res.bits);

  lastCmdSentMs = millis();

  if (res.decode_type == UNKNOWN) {
    // rawbuf holds capture ticks, sendRaw wants microseconds
    uint16_t len = getCorrectedRawLength(&res);
    uint16_t* raw = resultToRawArray(&res);
    if (!raw) return false;
    irsend.sendRaw(raw, len, IR_FREQ_HZ);
    delete[] raw;
  } else if (hasACState(res.decode_type)) {
    irsend.send(res.decode_type, res.state, res.bits / 8);
  } else {
    irsend.send(res.decode_type, res.value, res.bits);
  }
  return true;
}

void wipeAll() {
  stopRecording(2);
  for (int i = 0; i < MAX_COMMANDS; i++) {
    deleteIR(i);
    cmds[i].exists  = false;
    cmds[i].name[0] = '\0';
  }
  activeId     = -1;
  pendingValid = false;
  prefs.begin(NVS_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  Serial.println("[CMD] wiped all commands");
}

// The list is the only burst of notifications this firmware sends. Pushing it
// faster than the connection interval overflows the stack queue and entries go
// missing, so it is paced. Pacing with delay() would stall loop() for most of a
// second on a full list, so it walks one entry per call instead.
void startList() {
  listCursor   = 0;
  lastListSend = millis() - LIST_PACE_MS;   // let the first entry go at once
}

void serviceList(uint32_t now) {
  if (listCursor < 0) return;
  if (!bleConnected) { listCursor = -1; return; }
  if (now - lastListSend < LIST_PACE_MS) return;

  while (listCursor < MAX_COMMANDS && !cmds[listCursor].exists) listCursor++;

  if (listCursor >= MAX_COMMANDS) {
    bleNotify1(EV_LIST_END);
    listCursor = -1;
    return;
  }

  uint8_t buf[4 + MAX_NAME_LEN];
  uint8_t nlen = strlen(cmds[listCursor].name);
  buf[0] = EV_LIST_ENTRY;
  buf[1] = (uint8_t)listCursor;
  buf[2] = (listCursor == activeId) ? 1 : 0;
  buf[3] = nlen;
  memcpy(&buf[4], cmds[listCursor].name, nlen);
  bleNotify(buf, 4 + nlen);

  lastListSend = now;
  listCursor++;
}

void notifyCapture(int dupId) {
  uint8_t buf[15];
  uint16_t proto = (uint16_t)pendingResult.decode_type;
  uint64_t value = pendingResult.value;
  buf[0] = EV_CAPTURE;
  buf[1] = (dupId >= 0) ? 1 : 0;
  buf[2] = (dupId >= 0) ? (uint8_t)dupId : 0xFF;
  buf[3] = proto & 0xFF;
  buf[4] = proto >> 8;
  buf[5] = pendingResult.bits & 0xFF;
  buf[6] = pendingResult.bits >> 8;
  memcpy(&buf[7], &value, 8);
  bleNotify(buf, sizeof(buf));
}

void notifySaved(int id) {
  uint8_t buf[3 + MAX_NAME_LEN];
  uint8_t nlen = strlen(cmds[id].name);
  buf[0] = EV_SAVED;
  buf[1] = (uint8_t)id;
  buf[2] = nlen;
  memcpy(&buf[3], cmds[id].name, nlen);
  bleNotify(buf, 3 + nlen);
}

// NeoPixel status ------------------------------------------------
float interpolatePercentage(float voltage) {
  if (voltage <= voltageLUT[0])
    return 0;
  if (voltage >= voltageLUT[lutSize - 1])
    return 100;
  int i = 0;
  while (i < lutSize - 1 && voltage > voltageLUT[i + 1])
    i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

int getCurrentBatteryPercentage() {
  // the divide below expects millivolts, so the fallback reads them too
  float avgRaw = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount)
                                       : analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  float voltage = (avgRaw / 1000.0) * 2;
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);
  if (lastBatteryPct == -1) {
    lastBatteryPct = (int)percentage;
  } else if ((int)percentage < lastBatteryPct) {
    lastBatteryPct = (int)percentage;
    risingCount = 0;
  } else if ((int)percentage > lastBatteryPct) {
    risingCount++;
    if (risingCount >= RISING_THRESHOLD) {
      lastBatteryPct = (int)percentage;
      risingCount = 0;
    }
  } else {
    risingCount = 0;
  }
  return lastBatteryPct;
}

// Turns a battery percentage into the color for the battery led.
uint32_t batteryPercentToColor(int percent) {
  if (percent <= 20) return pixel.Color(20, 0, 0);
  if (percent <= 70) return pixel.Color(35, 7, 0);
  return pixel.Color(0, 20, 0);
}

// Writes both leds and shows them, but only when a color changed.
void updateStatusLeds(bool connected, unsigned long nowMs) {
  // Bluetooth led: red until connected, blue while a command is firing,
  // green when connected but idle.
  if (nowMs - lastCmdSentMs < BLUE_LED_DURATION) {
    bleColor = pixel.Color(0, 0, 30);
  } else if (!connected) {
    bleColor = pixel.Color(20, 0, 0);
  } else {
    bleColor = pixel.Color(0, 20, 0);
  }

  if (bleColor == shownBleColor && batteryColor == shownBatteryColor) {
    return;
  }

  shownBleColor = bleColor;
  shownBatteryColor = batteryColor;

  pixel.setPixelColor(BLE_LED, bleColor);
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.show();
}

void startRecording() {
  if (recording) return;
  // a new recording supersedes a capture that was never named, so an
  // abandoned one can never block the button
  pendingValid = false;
  recording = true;
  recordAt  = millis();
  irrecv.enableIRIn();
  uint8_t buf[2] = { EV_LISTENING, RECORD_TIMEOUT / 1000 };
  bleNotify(buf, 2);
  Serial.println("[REC] listening");
}

// reason: 0 timed out, 1 cancelled by the button, 2 cancelled by the app
void stopRecording(uint8_t reason) {
  if (!recording) return;
  irrecv.disableIRIn();
  recording = false;
  uint8_t buf[2] = { EV_LISTEN_END, reason };
  bleNotify(buf, 2);
  Serial.printf("[REC] stopped, reason %d\n", reason);
}

void copyName(int id, const uint8_t* src, size_t len) {
  if (len > MAX_NAME_LEN) len = MAX_NAME_LEN;
  memcpy(cmds[id].name, src, len);
  cmds[id].name[len] = '\0';
}

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    connectedAt  = millis();
    reqList      = true;
    Serial.println("[BLE] connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    reqList      = false;
    // nothing can name a capture now, and only loop() may touch the receiver
    pendingValid = false;
    reqCancel    = true;
    Serial.println("[BLE] disconnected");
    s->getAdvertising()->start();
  }
};

class WriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    uint8_t* d = c->getData();
    size_t   n = c->getLength();
    if (n == 0) return;

    switch (d[0]) {

      case CMD_SET_ACTIVE: {
        if (n < 2) return;
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS || !cmds[id].exists) return;
        activeId = id;
        nvsSave();
        uint8_t buf[2] = { EV_ACTIVE, (uint8_t)id };
        bleNotify(buf, 2);
        return;
      }

      case CMD_DELETE: {
        if (n < 2) return;
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS || !cmds[id].exists) return;
        cmds[id].exists  = false;
        cmds[id].name[0] = '\0';
        deleteIR(id);
        if (activeId == id) activeId = -1;
        nvsSave();
        uint8_t buf[2] = { EV_DELETED, (uint8_t)id };
        bleNotify(buf, 2);
        return;
      }

      case CMD_RENAME: {
        if (n < 3) return;
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS || !cmds[id].exists) return;
        copyName(id, &d[2], n - 2);
        nvsSave();
        notifySaved(id);
        return;
      }

      case CMD_SAVE_NEW: {
        if (!pendingValid || n < 2) {
          Serial.println("[CMD] save new: nothing pending");
          bleNotify1(EV_FAIL);
          return;
        }
        int id = freeSlot();
        if (id < 0) {
          Serial.println("[CMD] save new: no free slot");
          bleNotify1(EV_FAIL);
          return;
        }
        if (!saveIR(id, &pendingResult)) { bleNotify1(EV_FAIL); return; }
        copyName(id, &d[1], n - 1);
        cmds[id].exists = true;
        if (activeId < 0) activeId = id;
        nvsSave();
        notifySaved(id);
        pendingValid = false;
        return;
      }

      case CMD_SAVE_OVER: {
        if (!pendingValid || n < 3) {
          Serial.println("[CMD] save over: nothing pending");
          bleNotify1(EV_FAIL);
          return;
        }
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS) {
          Serial.println("[CMD] save over: bad slot");
          bleNotify1(EV_FAIL);
          return;
        }
        // saveIR truncates, so the old waveform only goes once the new one is safe
        if (!saveIR(id, &pendingResult)) { bleNotify1(EV_FAIL); return; }
        copyName(id, &d[2], n - 2);
        cmds[id].exists = true;
        nvsSave();
        notifySaved(id);
        pendingValid = false;
        return;
      }

      case CMD_GET_LIST:   reqList   = true; return;
      case CMD_WIPE:       reqWipe   = true; return;
      case CMD_CANCEL_REC: reqCancel = true; return;

      case CMD_DISCARD:
        pendingValid = false;
        Serial.println("[CMD] capture discarded");
        return;

      case CMD_FIRE:
        if (n < 2) return;
        reqFire = d[1];
        return;
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[BOOT] IR Transceiver");

  pinMode(USER_BTN_PIN, INPUT_PULLUP);
  irsend.begin();

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pixel.begin();
  pixel.clear();
  pixel.show();

  int currentBattery = getCurrentBatteryPercentage();
  batteryColor = batteryPercentToColor(currentBattery);
  Serial.printf("[BATT] %d%%\n", currentBattery);

  if (!LittleFS.begin(true)) Serial.println("[FS] mount failed");

  updateStatusLeds(false, millis());   // battery on, bluetooth red
  lastBatteryCheck = millis();         // the battery was just read above

  nvsLoad();

  BLEDevice::init(BLE_NAME);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());

  BLEService* svc = srv->createService(SVC_UUID);
  pNotify = svc->createCharacteristic(NOTIFY_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pNotify->addDescriptor(new BLE2902());

  BLECharacteristic* pWrite = svc->createCharacteristic(
      WRITE_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pWrite->setCallbacks(new WriteCB());

  svc->start();
  srv->getAdvertising()->start();
  Serial.println("[BLE] advertising as " BLE_NAME);
}

// HELD means the hold threshold was reached while the button is still down,
// so the release that follows must not be treated as a short press.
enum BtnState { IDLE, PRESSED, HELD };
BtnState btnState    = IDLE;
uint32_t btnPressed  = 0;
uint32_t lastRelease = 0;

void loop() {
  uint32_t now = millis();

  if (now - lastBatterySample >= BATTERY_SAMPLE_MS) {
    lastBatterySample = now;
    batteryWinSum += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;
  }
  if (now - lastBatteryCheck >= BATTERY_CHECK_MS) {
    lastBatteryCheck = now;
    batteryColor = batteryPercentToColor(getCurrentBatteryPercentage());
  }
  // show() briefly disables interrupts, which can cost the receiver an edge,
  // so the ring is left alone while it is armed
  if (!recording) updateStatusLeds(bleConnected, now);

  // Deferred work from the BLE callback
  if (reqFire >= 0) {
    int id = reqFire;
    reqFire = -1;
    bleNotify1(fireCommand(id) ? EV_OK : EV_FAIL);
  }
  if (reqWipe) {
    reqWipe = false;
    wipeAll();
    bleNotify1(EV_WIPED);
  }
  if (reqCancel) {
    reqCancel = false;
    stopRecording(2);
  }
  if (reqList && bleConnected && (now - connectedAt) >= LIST_DELAY_MS) {
    reqList = false;
    startList();
  }
  serviceList(now);

  bool btnLow = (digitalRead(USER_BTN_PIN) == LOW);

  switch (btnState) {
    case IDLE:
      if (btnLow && (now - lastRelease) >= DEBOUNCE_MS) {
        btnState   = PRESSED;
        btnPressed = now;
      }
      break;

    case PRESSED:
      if (!btnLow) {
        lastRelease = now;
        btnState    = IDLE;
        // short press: cancel a recording if one is running, otherwise fire
        if (recording) stopRecording(1);
        else if (!fireCommand(activeId)) Serial.println("[BTN] nothing to fire");
      } else if ((now - btnPressed) >= HOLD_THRESHOLD) {
        btnState = HELD;
        startRecording();
      }
      break;

    case HELD:
      // recording stays latched once the button comes back up
      if (!btnLow) {
        lastRelease = now;
        btnState    = IDLE;
      }
      break;
  }

  if (recording && (now - recordAt) >= RECORD_TIMEOUT) stopRecording(0);

  if (recording && irrecv.decode(&pendingResult)) {
    irrecv.disableIRIn();
    recording = false;

    // rawbuf points into the receiver, copy it out before it is reused
    uint16_t len = min(pendingResult.rawlen, (uint16_t)RAW_BUF_LEN);
    for (uint16_t i = 0; i < len; i++) pendingRaw[i] = pendingResult.rawbuf[i];
    pendingResult.rawbuf = pendingRaw;
    pendingResult.rawlen = len;
    pendingValid = true;

    int dupId = findDuplicate(&pendingResult);
    Serial.printf("[IR] captured proto=%s bits=%d dup=%d\n",
                  typeToString(pendingResult.decode_type).c_str(),
                  pendingResult.bits, dupId);
    notifyCapture(dupId);
  }
}
