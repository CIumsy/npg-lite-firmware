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
// Command names live in NVS, captured waveforms in LittleFS. The active slot
// is a selection, not a setting, so it stays in RAM and is never written.

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




#define BIOAMP_ENABLED     true  // false = plain IR remote, no bio-potential data sampling at all
#define BIOAMP_CHANNELS    { 0 } // what channels to sample

#define NOTCH_HZ           50    // mains notch, 50 or 60 only
#define SAMPLE_RATE        500   // per channel, must match the filter design

#define JAW_THRESHOLD      75    // envelope level that starts a clench
#define JAW_RELEASE        60    // must fall below this before the next one counts
#define JAW_HOLD_MS        600   // a clench held this long starts scrolling down
#define JAW_REPEAT_MS      300   // scroll cadence while held, matches a fast tap
#define JAW_BLOCK_MS       500   // a clench swamps the EEG, so ignore focus around it

#define FOCUS_THRESHOLD    10.0f // beta share of total EEG power, percent
#define FOCUS_DEBOUNCE_MS  2000  // ignore further focus triggers for this long

#define TRIGGER_DEBOUNCE   300   // ignore new triggers for this long after one lands
#define TRIGGER_SETTLE     300   // ignore triggers after a reset, filters are ringing
#define STALL_GAP_MS       25    // a service gap longer than this means we were blocked

#define SAMPLE_BLOCK_COUNT 10    // samples per channel per DMA frame
#define BATTERY_PIN        6     // battery divider sits on ADC1 channel 6 (A6)

#if BIOAMP_ENABLED
  #include "freertos/FreeRTOS.h"
  #include "freertos/semphr.h"
  #include "esp_adc/adc_continuous.h"
  #include "hal/adc_types.h"
  #include "hal/efuse_hal.h"
  #include "soc/soc_caps.h"
  #include "Filters/Envelope.h"
  #include "Filters/EMGFilter.h"
  #include "Filters/EEGFilter.h"
  #include "Filters/BetaPower.h"

  static const uint8_t bioampChannel[] = BIOAMP_CHANNELS;
  #define CHANNEL_COUNT (sizeof(bioampChannel) / sizeof(bioampChannel[0]))

  // sizeof is invisible to the preprocessor, so this is a compile time
  // constant check rather than an #error
  static_assert(CHANNEL_COUNT >= 1 && CHANNEL_COUNT <= 6,
                "BIOAMP_CHANNELS must name between 1 and 6 channels");

  // Only two mains frequencies exist, so reject anything else at compile time
  // rather than silently running an unfiltered signal.
  #if NOTCH_HZ == 50
    #include "Filters/Notch50.h"
    typedef Notch50 NotchFilter;
  #elif NOTCH_HZ == 60
    #include "Filters/Notch60.h"
    typedef Notch60 NotchFilter;
  #else
    #error "NOTCH_HZ must be 50 or 60"
  #endif
#endif

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

static void nvsKey(char* out, size_t len, int id) {
  snprintf(out, len, "n%d", id);
}

void nvsLoad() {
  prefs.begin(NVS_NAMESPACE, true);
  for (int i = 0; i < MAX_COMMANDS; i++) {
    char key[8];
    nvsKey(key, sizeof(key), i);
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

  // The active slot is a selection, not a setting. Scrolling changes it
  // constantly, so it lives in RAM and never reaches flash. On boot start at
  // the first saved command so the button always has something to fire.
  activeId = -1;
  for (int i = 0; i < MAX_COMMANDS; i++) {
    if (cmds[i].exists) { activeId = i; break; }
  }
}

// One slot at a time. Renaming one command must not rewrite fifty keys.
void nvsSaveName(int id) {
  char key[8];
  nvsKey(key, sizeof(key), id);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putString(key, cmds[id].name);
  prefs.end();
}

void nvsRemoveName(int id) {
  char key[8];
  nvsKey(key, sizeof(key), id);
  prefs.begin(NVS_NAMESPACE, false);
  prefs.remove(key);
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

#if BIOAMP_ENABLED
uint16_t bioampBatteryReading();   // defined with the sampling code below
#endif

// Averaged reading for the battery divider. With EMG enabled the continuous
// driver owns ADC1, so the value arrives through the DMA pattern instead of a
// one shot read. On the C6 a raw count is close enough to a millivolt for the
// curve below, which is how the other NPG Lite firmware handles it too.
static float batteryReading() {
#if BIOAMP_ENABLED
  uint16_t avg = bioampBatteryReading();
  return (avg > 0) ? (float)avg : 0.0f;
#else
  float avg = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount)
                                    : analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
  batteryWinSum = 0;
  batteryWinCount = 0;
  return avg;
#endif
}

int getCurrentBatteryPercentage() {
  float avgRaw = batteryReading();
  if (avgRaw <= 0.0f) return (lastBatteryPct < 0) ? 100 : lastBatteryPct;
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
        nvsRemoveName(id);
        uint8_t buf[2] = { EV_DELETED, (uint8_t)id };
        bleNotify(buf, 2);
        return;
      }

      case CMD_RENAME: {
        if (n < 3) return;
        int id = d[1];
        if (id < 0 || id >= MAX_COMMANDS || !cmds[id].exists) return;
        copyName(id, &d[2], n - 2);
        nvsSaveName(id);
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
        nvsSaveName(id);
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
        nvsSaveName(id);
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

#if BIOAMP_ENABLED
// ===========================================================================
// EMG sampling and triggers
//
// The ADC runs in continuous DMA mode, so samples are taken by hardware and
// never jitter, no matter what loop() is doing. Everything else in this
// firmware blocks at some point: flash writes, IR transmission, the list
// stream. Rather than try to keep filtering through those, bioampService spots
// the gap, throws away what DMA collected, and resets the filters. That keeps
// the signal honest at the cost of a short blind window.
// ===========================================================================

#define ADC_PATTERN_LEN  (CHANNEL_COUNT + 1)   // bioamp channels plus battery
#define ADC_BATTERY_IDX  CHANNEL_COUNT         // battery is last in the pattern
#define ADC_FRAME_BYTES  (ADC_PATTERN_LEN * SOC_ADC_DIGI_RESULT_BYTES * SAMPLE_BLOCK_COUNT)

static adc_continuous_handle_t adcHandle = nullptr;
static SemaphoreHandle_t       adcSem    = nullptr;
static bool                    bioampReady  = false;

// Mains notch is the only thing applied to every channel, because it is always
// wanted. Each channel's latest notched sample is parked in notched[] and
// anything further is per channel and left to you. See the sample loop.
static NotchFilter notchFilter[CHANNEL_COUNT];
static float       notched[CHANNEL_COUNT];

// Channel 0 is the one wired to the triggers. Its notched signal is split two
// ways: high passed and enveloped for the jaw clench, and low passed into a
// spectrum for the beta share that detects focus.
static EMGFilter   emgFilter;
static Envelope    envelope;
static int         jawLevel = 0;
static EEGFilter   eegFilter;
static BetaPower   betaPower;
static bool        jawHeld      = false;
static bool        jawScrolling = false;
static uint32_t    jawStartMs   = 0;
static uint32_t    lastRepeatMs = 0;
static uint32_t    lastJawMs    = 0;
static uint32_t    lastFocusMs  = 0;

// maps a physical ADC channel back to its slot in the pattern
static int8_t adcChannelIndex[SOC_ADC_CHANNEL_NUM(0)];

static uint32_t lastServiceMs = 0;
static uint32_t settleUntil = 0;

static uint32_t battWinSum   = 0;
static uint16_t battWinCount = 0;

// The C6 rev1 ADC tops out below full scale, so stretch it back.
static inline uint16_t fixRaw(uint16_t raw) {
  static uint32_t chiprev = efuse_hal_chip_revision();
  if (chiprev == 1) {
    uint32_t v = (uint32_t)raw * 4095u / 3249u;
    return (uint16_t)(v > 4095u ? 4095u : v);
  }
  return raw;
}

static bool IRAM_ATTR adcOnConvDone(adc_continuous_handle_t handle,
                                    const adc_continuous_evt_data_t* edata,
                                    void* user_data) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(adcSem, &woken);
  return woken == pdTRUE;
}

bool bioampBegin() {
  adcSem = xSemaphoreCreateBinary();
  if (!adcSem) return false;

  static adc_digi_pattern_config_t pattern[ADC_PATTERN_LEN];
  for (int i = 0; i < ADC_PATTERN_LEN; i++) {
    pattern[i].atten     = ADC_ATTEN_DB_12;
    pattern[i].channel   = (i == ADC_BATTERY_IDX) ? BATTERY_PIN : bioampChannel[i];
    pattern[i].unit      = ADC_UNIT_1;
    pattern[i].bit_width = ADC_BITWIDTH_12;
  }
  for (size_t i = 0; i < sizeof(adcChannelIndex); i++) adcChannelIndex[i] = -1;
  for (int i = 0; i < ADC_PATTERN_LEN; i++) adcChannelIndex[pattern[i].channel] = i;

  adc_continuous_handle_cfg_t handleCfg = {
    .max_store_buf_size = ADC_FRAME_BYTES * 4,
    .conv_frame_size    = ADC_FRAME_BYTES,
  };
  if (adc_continuous_new_handle(&handleCfg, &adcHandle) != ESP_OK) return false;

  adc_continuous_evt_cbs_t cbs = { .on_conv_done = adcOnConvDone };
  if (adc_continuous_register_event_callbacks(adcHandle, &cbs, nullptr) != ESP_OK) return false;

  adc_continuous_config_t cfg = {
    .pattern_num    = ADC_PATTERN_LEN,
    .adc_pattern    = pattern,
    .sample_freq_hz = (uint32_t)(SAMPLE_RATE * ADC_PATTERN_LEN),
    .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
    .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  if (adc_continuous_config(adcHandle, &cfg) != ESP_OK) return false;
  if (adc_continuous_start(adcHandle) != ESP_OK) return false;

  betaPower.begin((float)SAMPLE_RATE);

  bioampReady = true;
  Serial.printf("[BIO] %d channel(s) at %d Hz, %d Hz notch\n",
                (int)CHANNEL_COUNT, SAMPLE_RATE, NOTCH_HZ);
  return true;
}

// Throw away buffered samples and restart the filters. Called whenever
// something blocked us, because a gap mid-stream makes the IIR state
// meaningless and the ringing on resume looks exactly like a contraction.
static void bioampReset() {
  if (adcHandle) {
    uint8_t scratch[ADC_FRAME_BYTES];
    uint32_t got = 0;
    while (adc_continuous_read(adcHandle, scratch, sizeof(scratch), &got, 0) == ESP_OK && got) {}
  }
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    notchFilter[i].reset();
    notched[i] = 0.0f;
  }
  emgFilter.reset();
  envelope.reset();
  jawLevel = 0;
  eegFilter.reset();
  betaPower.reset();
  jawHeld      = false;
  jawScrolling = false;
  settleUntil  = millis() + TRIGGER_SETTLE;
}

// next occupied slot in the given direction, wrapping, -1 if the list is empty
static int nextSlot(int from, int dir) {
  for (int k = 1; k <= MAX_COMMANDS; k++) {
    int i = ((from + dir * k) % MAX_COMMANDS + MAX_COMMANDS) % MAX_COMMANDS;
    if (cmds[i].exists) return i;
  }
  return -1;
}

static void stepCommand(int dir) {
  int from = (activeId >= 0) ? activeId : (dir > 0 ? MAX_COMMANDS - 1 : 0);
  int next = nextSlot(from, dir);
  if (next < 0 || next == activeId) return;

  activeId = next;

  uint8_t buf[2] = { EV_ACTIVE, (uint8_t)activeId };
  bleNotify(buf, 2);
  Serial.printf("[EMG] %s -> slot %d\n", dir > 0 ? "next" : "prev", activeId);
}

// One channel, three gestures.
//   tap    a short clench steps up one slot
//   hold   a sustained clench scrolls down, repeating at the tap cadence
//   focus  sustained beta fires the active command
//
// A tap can only be told apart from a hold once the muscle relaxes, so the
// step happens on release. A clench also floods the EEG band, which is why
// focus is ignored while one is in progress and for a moment afterwards.
static void evaluateTriggers(uint32_t now) {
  int level = jawLevel;

  if (!jawHeld) {
    if (level > JAW_THRESHOLD && (now - lastJawMs) >= TRIGGER_DEBOUNCE) {
      jawHeld      = true;
      jawScrolling = false;
      jawStartMs   = now;
    }
  } else if (level < JAW_RELEASE) {
    // hysteresis, the muscle has to relax before the next clench counts
    jawHeld   = false;
    lastJawMs = now;
    if (!jawScrolling) stepCommand(-1);    // it was a tap, so step up
    jawScrolling = false;
  } else if (!jawScrolling && (now - jawStartMs) >= JAW_HOLD_MS) {
    jawScrolling = true;               // held long enough, start scrolling down
    lastRepeatMs = now;
    stepCommand(+1);
  } else if (jawScrolling && (now - lastRepeatMs) >= JAW_REPEAT_MS) {
    lastRepeatMs = now;
    stepCommand(+1);
  }

  bool jawNoise = jawHeld || (now - lastJawMs) < JAW_BLOCK_MS;
  if (!jawNoise && betaPower.value() > FOCUS_THRESHOLD &&
      (now - lastFocusMs) >= FOCUS_DEBOUNCE_MS) {
    lastFocusMs = now;
    reqFire = activeId;
    Serial.printf("[EEG] focus %.1f%% -> fire\n", betaPower.value());
  }
}

void bioampService() {
  if (!bioampReady) return;

  uint32_t now = millis();
  if (lastServiceMs && (now - lastServiceMs) > STALL_GAP_MS) bioampReset();
  lastServiceMs = now;

  if (xSemaphoreTake(adcSem, 0) != pdTRUE) return;

  uint8_t frame[ADC_FRAME_BYTES];
  uint32_t len = 0;
  while (adc_continuous_read(adcHandle, frame, sizeof(frame), &len, 0) == ESP_OK && len) {
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= len; i += SOC_ADC_DIGI_RESULT_BYTES) {
      auto* p = (const adc_digi_output_data_t*)&frame[i];
      uint8_t hw = p->type2.channel;
      if (hw >= sizeof(adcChannelIndex)) continue;
      int8_t idx = adcChannelIndex[hw];
      if (idx < 0) continue;

      if (idx == ADC_BATTERY_IDX) {
        battWinSum += p->type2.data;
        battWinCount++;
      } else {
        // notch every channel, each with its own filter state
        notched[idx] = notchFilter[idx].process(fixRaw(p->type2.data));

        if (idx == 0) {
          float muscle = emgFilter.process(notched[0]);
          jawLevel = envelope.process(abs((int)muscle));
          betaPower.push(eegFilter.process(notched[0]));
        }

        // Adding a channel? Its notched sample is ready in notched[idx] right
        // here. Declare your own filter and envelope for it above, process it
        // in this block, then act on the result in evaluateTriggers().
      }
    }
  }

  if (now >= settleUntil) evaluateTriggers(now);
}

// Averaged battery reading in the same units the LED code expects.
// Returns 0 when no samples have arrived yet.
uint16_t bioampBatteryReading() {
  if (battWinCount == 0) return 0;
  uint16_t avg = (uint16_t)(battWinSum / battWinCount);
  battWinSum   = 0;
  battWinCount = 0;
  return avg;
}
#endif  // BIOAMP_ENABLED

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[BOOT] IR Transceiver");

  pinMode(USER_BTN_PIN, INPUT_PULLUP);
  irsend.begin();

#if !BIOAMP_ENABLED
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
#endif
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

#if BIOAMP_ENABLED
  if (!bioampBegin()) Serial.println("[EMG] sampling failed to start");
#endif
}

// HELD means the hold threshold was reached while the button is still down,
// so the release that follows must not be treated as a short press.
enum BtnState { IDLE, PRESSED, HELD };
BtnState btnState    = IDLE;
uint32_t btnPressed  = 0;
uint32_t lastRelease = 0;

void loop() {
  uint32_t now = millis();

#if BIOAMP_ENABLED
  bioampService();
#else
  if (now - lastBatterySample >= BATTERY_SAMPLE_MS) {
    lastBatterySample = now;
    batteryWinSum += analogReadMilliVolts(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;
  }
#endif
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
