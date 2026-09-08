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

// Copyright (c) 2024-2025 Aman Maheshwari    - Aman@upsidedownlabs.tech
// Copyright (c) 2024-2025 Deepak Khatri      - deepak@upsidedownlabs.tech
// Copyright (c) 2024-2025 Upside Down Labs   - contact@upsidedownlabs.tech
//
// NPG Lite BCI firmware, retargeted to drive a stock/unmodified
// microbots.io ProtoBot directly over BLE - NO firmware changes needed
// on the ProtoBot side.
//
// HOW THIS WORKS:
// ProtoBot ships running the CodeCell-MicroLink library, which exposes
// a BLE GATT "joystick" characteristic that the MicroLink phone app
// normally writes to for driving it. This board now acts as a BLE
// CLIENT that connects directly to that same characteristic and writes
// joystick values itself - so from ProtoBot's perspective it looks
// exactly like the MicroLink app is driving it.
//
// Protocol reverse-derived from the public CodeCell-MicroLink source
// (github.com/microbotsio/CodeCell-MicroLink, src/MicroLink.h/.cpp):
//   Service UUID:     12345678-1234-1234-1234-123456789012
//   Joystick char UUID: abcd1234-abcd-1234-abcd-123456789012 (WRITE)
//   Settings char UUID: dcba4330-dcba-4321-dcba-432123456789 (WRITE)
//   Value = 2 bytes [X, Y], range 0-200, 100 = center/neutral.
//     X = steering/direction, Y = throttle/speed (per library comments)
//   Device advertises as "CodeCell".
//
// NOTE: X/Y -> actual turn direction is inferred from the library's
// variable naming/comments, not from an official spec - verify on the
// bench and flip JOY_LEFT/JOY_RIGHT below if it turns the wrong way.
//
// -----------------------------------------------------------------
// THIS VERSION:
//   - All three analog inputs (A0, A1, A2) are treated as EMG
//     channels 
//   - Channel 1 (A0) is the "forward" channel: its EMG envelope
//     crossing its threshold drives the bot forward.
//   - Channel 2 (A1) drives right turn, Channel 3 (A2) drives left
//     turn, and both together (at half-threshold) drive backward -
//     same as before.
//   - Forward/Backward/Stop is now an edge-triggered, debounced
//     3-state cycle: 0 (stop) -> 4 (forward) -> 3 (backward) -> 0 ...
//     each time all three EMG channels cross their (high) thresholds
//     together. Left/Right turning works as an independent overlay on
//     top of whatever fwd/back/stop state is currently active, and
//     resumes that state automatically once the turn is released.
//   - NEW: this board now ALSO runs a small BLE "config" GATT server
//     (peripheral role) alongside its existing BLE client role, so a
//     companion web page (Web Bluetooth) can read the current
//     thresholds and set new ones live. New values are written
//     straight to NVS (Preferences) - same storage the serial
//     "set ..." commands already used, so both control paths share
//     one source of truth.
// -----------------------------------------------------------------

#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <vector>
#include <Preferences.h>

// ---------------------------------------------------------------
//  ProtoBot / MicroLink BLE protocol (client side)
// ---------------------------------------------------------------
// This is the UUID ProtoBot actually broadcasts in its BLE advertising
// packet (confirmed via LightBlue scan) - used ONLY to find/filter the
// device while scanning.
static BLEUUID protobotAdvertisedUUID("00008018-0000-1000-8000-00805F9B34FB");

// This is the real GATT service that contains the joystick/settings
// characteristics, only visible AFTER connecting (confirmed via
// LightBlue's connected service browser - matches the public
// CodeCell-MicroLink library source exactly).
static BLEUUID joystickServiceUUID("12345678-1234-1234-1234-123456789012");
static BLEUUID joystickCharUUID   ("abcd1234-abcd-1234-abcd-123456789012");
static BLEUUID settingsCharUUID   ("dcba4330-dcba-4321-dcba-432123456789");
const char* PROTOBOT_DEVICE_NAME = "protobot";

// ---------------------------------------------------------------
//  Threshold config BLE service (this board acts as a PERIPHERAL/
//  SERVER for this one) - lets a companion web page (Web Bluetooth)
//  read/set EMG thresholds live, persisted to NVS (Preferences).
//
//  Each gesture is a combination of channels, and the same physical
//  channel can need a DIFFERENT trigger level depending on which
//  gesture it's part of - so thresholds are keyed per (gesture,
//  channel) pair, not just per channel. There are 5 of them:
//
//    id 0  thrFbsCh1   - Fwd/Back/Stop cycle, channel 1 (A0) leg
//    id 1  thrFbsCh3   - Fwd/Back/Stop cycle, channel 3 (A2) leg
//    id 2  thrLeftCh1  - Left-turn gesture,   channel 1 (A0) leg
//    id 3  thrLeftCh2  - Left-turn gesture,   channel 2 (A1) leg
//    id 4  thrRightCh3 - Right-turn gesture,  channel 3 (A2) leg
//
//  Write characteristic protocol: 5 bytes per write
//    [0]    thresholdId (0-4, see above)
//    [1..4] uint32, little-endian, new threshold value
//
//  Read/notify characteristic: 20 bytes, always the current values,
//  in the same id order as above (5 x uint32 LE):
//    [0..3]   thrFbsCh1
//    [4..7]   thrFbsCh3
//    [8..11]  thrLeftCh1
//    [12..15] thrLeftCh2
//    [16..19] thrRightCh3
//
//  Live EMG notify characteristic: 6 bytes, sent at ~51 Hz (throttled
//  down from the 512 Hz sample loop) ONLY while a config client is
//  connected, so a web page can plot the live envelope traces next to
//  the threshold sliders. Also READABLE on demand (always returns a
//  fresh sample) as a fallback for clients where GATT notify delivery
//  is unreliable. 3 x uint16 LE, raw envelope units (same
//  scale as the thresholds above):
//    [0..1] channel 1 (A0) envelope
//    [2..3] channel 2 (A1) envelope
//    [4..5] channel 3 (A2) envelope
// ---------------------------------------------------------------
static BLEUUID configServiceUUID  ("d1b3d900-0000-1000-8000-00805f9b34fb");
static BLEUUID thresholdWriteUUID ("d1b3d901-0000-1000-8000-00805f9b34fb");
static BLEUUID thresholdReadUUID  ("d1b3d902-0000-1000-8000-00805f9b34fb");
static BLEUUID emgLiveUUID        ("d1b3d903-0000-1000-8000-00805f9b34fb");

// Joystick value range/semantics (from MicroLink source: 0-200, 100=center)
const uint8_t JOY_CENTER = 100;
const uint8_t JOY_FWD    = 0;    // Y above center = forward
const uint8_t JOY_BWD    = 200;  // Y below center = backward
const uint8_t JOY_LEFT   = 0;    // X below center = left  (flip with JOY_RIGHT if reversed)
const uint8_t JOY_RIGHT  = 200;  // X above center = right

// ---------------------------------------------------------------
//  Eye/display colors per direction - tune to taste (R,G,B 0-255)
// ---------------------------------------------------------------
const uint8_t COLOR_STOP[3]      = { 255, 255, 255 };  // white
const uint8_t COLOR_FORWARD[3]   = { 0,   255, 0   };  // green
const uint8_t COLOR_BACKWARD[3]  = { 255, 0,   0   };  // red
const uint8_t COLOR_LEFT[3]      = { 255, 200, 0   };  // amber/yellow
const uint8_t COLOR_RIGHT[3]     = { 255, 200, 0   };  // amber/yellow

// ---------------------------------------------------------------
//  Hardware pins
// ---------------------------------------------------------------
#define PIN_NEOPIXEL 15

// Hold this button for BOOT_HOLD_MS to toggle Config Mode (see below).
// GPIO9 is this board's actual BOOT/user button (confirmed from the
// board's own OTA-mode sketch), active-low with internal pull-up.
#define BOOT_BUTTON_PIN 9
#define BOOT_HOLD_MS     1000

Adafruit_NeoPixel pixel(6, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#define BLE_LED 0
#define BATTERY_LED 5

// ---------------------------------------------------------------
//  Signal processing config
// ---------------------------------------------------------------
#define SAMPLE_RATE 512
#define BAUD_RATE 115200
#define INPUT_PIN1 A0           // EMG - forward
#define INPUT_PIN2 A1           // EMG - right
#define INPUT_PIN3 A2           // EMG - left
#define BATTERY_VOLTAGE_PIN A6  // Battery connected ADC
#define BLUE_LED_DURATION 100

// ---------------------------------------------------------------
//  Debug print rate: print every N samples (SAMPLE_RATE / N = print Hz)
// ---------------------------------------------------------------
#define DEBUG_PRINT_EVERY_N_SAMPLES 100

// ---------------------------------------------------------------
//  Thresholds - loaded from NVS on boot
// ---------------------------------------------------------------
// Five independent thresholds, one per (gesture, channel) leg - see
// the config-service comment above for what each one gates. The
// original version of this sketch used hardcoded trigger levels in
// loop() instead of variables at all (250/150 for channel 1 depending
// on gesture, 300/150 for channel 3 depending on gesture). Those two
// numbers per channel map directly onto the two variables below that
// share that channel, so to reproduce the old feel exactly, set:
// thrFbsCh1=250, thrLeftCh1=150, thrFbsCh3=300, thrRightCh3=150.
uint32_t thrFbsCh1   = 150;  // Fwd/Back/Stop cycle - channel 1 (A0) leg
uint32_t thrFbsCh3   = 150;  // Fwd/Back/Stop cycle - channel 3 (A2) leg
uint32_t thrLeftCh1  = 150;  // Left-turn gesture   - channel 1 (A0) leg
uint32_t thrLeftCh2  = 150;  // Left-turn gesture   - channel 2 (A1) leg
uint32_t thrRightCh3 = 150;  // Right-turn gesture  - channel 3 (A2) leg

Preferences prefs;

// ---------------------------------------------------------------
//  Globals shared between loop() and debug
// ---------------------------------------------------------------
float gEnv0 = 0.0f;  // forward channel envelope
float gEnv1 = 0.0f;  // right channel envelope
float gEnv2 = 0.0f;  // left channel envelope
bool debugEnabled = false;
static bool pixelDirty = false;

// ---------------------------------------------------------------
//  Config Mode - toggled by a 1s hold of the BOOT button (or the
//  serial "config" command). This board can't reliably run both BLE
//  roles at full tilt at once: connected-as-client-to-ProtoBot AND
//  streaming live EMG as a peripheral-server contend for the same
//  radio, which is why the live envelope was showing up unreliably
//  on the config page even though everything else "worked". So:
//    - Config Mode ON  -> pause/drop the ProtoBot link, stop scanning,
//      advertise the config service, stream live EMG to the web UI.
//    - Config Mode OFF (normal/default) -> resume driving ProtoBot as
//      before, and stop streaming live EMG / advertising so nothing
//      competes with the joystick link.
// ---------------------------------------------------------------
bool configMode = false;
static bool bootButtonWasDown = false;
static unsigned long bootButtonDownAtMs = 0;

// ---- Forward/Backward/Stop 3-state cycle + Left/Right overlay state ----
// fbState persists whichever of {0=stop, 4=forward, 3=backward} is
// currently "active" in the background, independent of any ongoing turn.
// Declared here (not just above loop()) so enterConfigMode() can reset
// them when the ProtoBot link is paused.
static uint32_t fbState = 0;
static bool fbConditionActive = false;     // edge-tracking for the triple threshold
static bool lrActive = false;              // true while left or right is currently held

// ---------------------------------------------------------------
//  BLE client state (connects OUT to ProtoBot)
// ---------------------------------------------------------------
static BLEAdvertisedDevice*     myDevice        = nullptr;
static BLERemoteCharacteristic* pJoystickChar   = nullptr;
static BLERemoteCharacteristic* pSettingsChar   = nullptr;
static BLEClient*               pClient         = nullptr;

static boolean doConnect = false;
static boolean doScan    = false;
bool deviceConnected     = false;   // true once connected to ProtoBot

// ---------------------------------------------------------------
//  BLE server state (config service - this board acts as PERIPHERAL
//  here, a separate role from the client above used to drive ProtoBot)
// ---------------------------------------------------------------
static BLEServer*         pConfigServer   = nullptr;
static BLECharacteristic* pThreshWriteChr = nullptr;
static BLECharacteristic* pThreshReadChr  = nullptr;
static BLECharacteristic* pEmgLiveChr     = nullptr;

// ---------------------------------------------------------------
//  Control state
// ---------------------------------------------------------------
bool stage = false;
uint32_t lastSentCmd = 255;

// ── BLE LED state machine ──
enum LedState {
  LED_RED,
  LED_GREEN,
  LED_BLUE_FADE
};
LedState ledState = LED_RED;
unsigned long lastCmdSentMs = 0;
uint32_t lastPixel0Color = 0xFFFFFFFF;

// ---------------------------------------------------------------
//  Battery level indication
// ---------------------------------------------------------------
static const unsigned long BATTERY_CHECK_INTERVAL = 10000;
static unsigned long lastBatteryCheck = 0;

uint32_t batteryColor = 0;
static uint32_t batteryWinSum = 0;
static uint16_t batteryWinCount = 0;
static int lastBatteryPct = -1;
static uint8_t risingCount = 0;
static const uint8_t RISING_THRESHOLD = 3;
const float voltageLUT[] = {
  3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82,
  3.84, 3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20
};

const int percentLUT[] = {
  0, 5, 10, 15, 20, 25, 30, 35, 40, 45,
  50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};

const int lutSize = sizeof(voltageLUT) / sizeof(voltageLUT[0]);

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
  float avgRaw = (batteryWinCount > 0) ? (batteryWinSum / batteryWinCount) : analogRead(BATTERY_VOLTAGE_PIN);
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

// ----------------- NOTCH FILTER CLASSES -----------------
class NotchFilter {
  struct BiquadState {
    float z1 = 0, z2 = 0;
  };
  BiquadState s1, s2;

public:
  float process(float in) {
    float x = in - (-1.56858163f * s1.z1) - (0.96424138f * s1.z2);
    float out = 0.96508099f * x + (-1.56202714f * s1.z1) + (0.96508099f * s1.z2);
    s1.z2 = s1.z1;
    s1.z1 = x;
    x = out - (-1.61100358f * s2.z1) - (0.96592171f * s2.z2);
    out = 1.0f * x + (-1.61854514f * s2.z1) + (1.0f * s2.z2);
    s2.z2 = s2.z1;
    s2.z1 = x;
    return out;
  }
  void reset() {
    s1.z1 = s1.z2 = s2.z1 = s2.z2 = 0;
  }
};

class EMGHighPassFilter {
  double z1 = 0, z2 = 0;

public:
  double process(double in) {
    double x = in - (-0.82523238) * z1 - (0.29463653) * z2;
    double out = 0.52996723 * x + (-1.05993445) * z1 + 0.52996723 * z2;
    z2 = z1;
    z1 = x;
    return out;
  }
  void reset() {
    z1 = z2 = 0;
  }
};

class EnvelopeFilter {
  std::vector<double> buf;
  double sum = 0;
  int idx = 0;
  const int sz;

public:
  EnvelopeFilter(int s)
    : sz(s) {
    buf.resize(s, 0.0);
  }
  double getEnvelope(double v) {
    sum -= buf[idx];
    sum += v;
    buf[idx] = v;
    idx = (idx + 1) % sz;
    return sum / sz;
  }
};

// Three EMG channels, all treated identically now.
NotchFilter filters[3];
EMGHighPassFilter emgfilters[3];
EnvelopeFilter Envelopefilter0(16);  // forward channel (A0)
EnvelopeFilter Envelopefilter1(16);  // right channel   (A1)
EnvelopeFilter Envelopefilter2(16);  // left channel    (A2)

// ---------------------------------------------------------------
//  Forward declarations
// ---------------------------------------------------------------
void printHelp();
void toggleConfigMode();

// ---------------------------------------------------------------
//  Joystick write helper - talks directly to ProtoBot's stock
//  MicroLink joystick characteristic
// ---------------------------------------------------------------
void writeJoystick(uint8_t x, uint8_t y) {
  if (!deviceConnected || pJoystickChar == nullptr) return;
  uint8_t buf[2] = { x, y };
  pJoystickChar->writeValue(buf, 2, false);  // write WITHOUT response - matches official ProtoBot joystick.ino example
}

// ---------------------------------------------------------------
//  Eye/display color helper - writes the full 10-byte settings
//  packet (same fields as before) but with the given RGB color.
// ---------------------------------------------------------------
void updateEyeColor(uint8_t r, uint8_t g, uint8_t b) {
  if (pSettingsChar == nullptr) return;
  uint8_t settings[10] = {
    1,     // [0] isAppConnectionReady
    0,     // [1] avoidOnOFF
    0,     // [2] FrontAvoidOnOFF
    0,     // [3] faceNorthOnOFF
    0xFF,  // [4] displayOnOFF
    r,     // [5] displayRed
    g,     // [6] displayGreen
    b,     // [7] displayBlue
    3,     // [8] speed_reduction (3 -> full speed)
    0      // [9] touchGesture
  };
  pSettingsChar->writeValue(settings, sizeof(settings), false);
}

// Maps a cmd (0-4) to its display color
void colorForCommand(uint32_t cmd, uint8_t &r, uint8_t &g, uint8_t &b) {
  const uint8_t *c;
  switch (cmd) {
    case 1:  c = COLOR_LEFT;     break;
    case 2:  c = COLOR_RIGHT;    break;
    case 3:  c = COLOR_FORWARD;  break;
    case 4:  c = COLOR_BACKWARD; break;
    default: c = COLOR_STOP;     break;
  }
  r = c[0]; g = c[1]; b = c[2];
}

// Maps a cmd (0-4) to its joystick X/Y pair
void joystickForCommand(uint32_t cmd, uint8_t &x, uint8_t &y) {
  switch (cmd) {
    case 1:  x = JOY_LEFT;   y = JOY_CENTER; break;  // turn left
    case 2:  x = JOY_RIGHT;  y = JOY_CENTER; break;  // turn right
    case 3:  x = JOY_CENTER; y = JOY_FWD;    break;  // forward
    case 4:  x = JOY_CENTER; y = JOY_BWD;    break;  // backward
    default: x = JOY_CENTER; y = JOY_CENTER; break;  // stop
  }
}

// ---------------------------------------------------------------
//  Continuous joystick refresh
//
//  Some BLE-controlled robots (ProtoBot included, based on observed
//  behavior) apply a failsafe: if no fresh joystick write arrives
//  within a short window, they auto-stop (in case the controller app
//  disconnects mid-drive). A single write-on-change is NOT enough -
//  we must keep re-sending the current command at a steady rate for
//  as long as it's active, or the bot will randomly stop.
// ---------------------------------------------------------------
uint32_t currentCommand = 0;
uint32_t lastLoggedCmd = 255;
unsigned long lastJoystickSendMs = 0;
const unsigned long JOYSTICK_REFRESH_MS = 100;  // resend at ~10 Hz

void sendJoystickNow() {
  uint8_t x, y;
  joystickForCommand(currentCommand, x, y);
  writeJoystick(x, y);
  lastJoystickSendMs = millis();
}

// applyCommand - only writes over BLE when the command actually
// CHANGES (critical: this gets called at up to SAMPLE_RATE Hz from
// the EMG sample loop while a threshold is held, so writing on every
// call floods the BLE stack and eventually jams it - which is why the
// robot appeared to "freeze" on the last command after a few
// seconds). The periodic ~10Hz refresh in loop() is solely
// responsible for keeping the command alive/repeated over time.
void applyCommand(uint32_t cmd) {
  currentCommand = cmd;

  if (cmd == lastLoggedCmd)
    return;  // no change -> no write, let the periodic refresh handle repetition

  lastLoggedCmd = cmd;
  lastSentCmd = cmd;
  Serial.print("cmd: ");
  Serial.println(cmd);
  lastCmdSentMs = millis();
  ledState = LED_BLUE_FADE;

  sendJoystickNow();

  uint8_t r, g, b;
  colorForCommand(cmd, r, g, b);
  updateEyeColor(r, g, b);
}

// ---------------------------------------------------------------
//  Threshold config: shared source of truth for BLE + serial
// ---------------------------------------------------------------

// Pushes one throttled sample of the 3 live EMG envelopes out over
// notify, for a web page's live trace view. Only called (from loop())
// while a config client is actually connected and subscribed.
void sendEmgLiveSample() {
  if (!pEmgLiveChr) return;
  uint16_t buf[3];
  buf[0] = (uint16_t)constrain((long)gEnv0, 0, 65535);
  buf[1] = (uint16_t)constrain((long)gEnv1, 0, 65535);
  buf[2] = (uint16_t)constrain((long)gEnv2, 0, 65535);
  pEmgLiveChr->setValue((uint8_t*)buf, sizeof(buf));  // ESP32 is little-endian, matches the JS DataView reads
  pEmgLiveChr->notify();
}

// Belt-and-suspenders for the live envelope characteristic: some BLE
// stack/client combinations are flaky about actually delivering GATT
// notifications even when the subscription itself succeeds, so on top
// of the periodic notify() above, this fills in a fresh reading
// on-demand whenever a client does an explicit READ (which is what a
// polling fallback in the web page uses, and which is known-reliable
// since the threshold characteristic already relies on it too).
class EmgLiveReadCallbacks : public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic* pChar) {
    uint16_t buf[3];
    buf[0] = (uint16_t)constrain((long)gEnv0, 0, 65535);
    buf[1] = (uint16_t)constrain((long)gEnv1, 0, 65535);
    buf[2] = (uint16_t)constrain((long)gEnv2, 0, 65535);
    pChar->setValue((uint8_t*)buf, sizeof(buf));
  }
};

// Pushes current thresholds into the read/notify characteristic so a
// freshly-connected (or already-connected) web UI picks up live
// values - including right after a serial "set" command.
void updateThresholdReadChr() {
  if (!pThreshReadChr) return;
  uint8_t buf[20];
  memcpy(buf,      &thrFbsCh1,   4);
  memcpy(buf + 4,  &thrFbsCh3,   4);
  memcpy(buf + 8,  &thrLeftCh1,  4);
  memcpy(buf + 12, &thrLeftCh2,  4);
  memcpy(buf + 16, &thrRightCh3, 4);
  pThreshReadChr->setValue(buf, 20);
  pThreshReadChr->notify();
}

// Applies + persists a threshold coming from either BLE or serial,
// so both control paths share one source of truth in NVS. id is one
// of the 5 (gesture, channel) legs - see the config-service comment.
void setThresholdById(uint8_t id, uint32_t val) {
  prefs.begin("thresholds", false);
  switch (id) {
    case 0:
      thrFbsCh1 = val;
      prefs.putUInt("fbsCh1thr", thrFbsCh1);
      Serial.print("thrFbsCh1 (Fwd/Back/Stop, ch1) set to ");
      Serial.println(thrFbsCh1);
      break;
    case 1:
      thrFbsCh3 = val;
      prefs.putUInt("fbsCh3thr", thrFbsCh3);
      Serial.print("thrFbsCh3 (Fwd/Back/Stop, ch3) set to ");
      Serial.println(thrFbsCh3);
      break;
    case 2:
      thrLeftCh1 = val;
      prefs.putUInt("leftCh1thr", thrLeftCh1);
      Serial.print("thrLeftCh1 (Left turn, ch1) set to ");
      Serial.println(thrLeftCh1);
      break;
    case 3:
      thrLeftCh2 = val;
      prefs.putUInt("leftCh2thr", thrLeftCh2);
      Serial.print("thrLeftCh2 (Left turn, ch2) set to ");
      Serial.println(thrLeftCh2);
      break;
    case 4:
      thrRightCh3 = val;
      prefs.putUInt("rightCh3thr", thrRightCh3);
      Serial.print("thrRightCh3 (Right turn, ch3) set to ");
      Serial.println(thrRightCh3);
      break;
    default:
      Serial.print("Unknown threshold id: ");
      Serial.println(id);
      prefs.end();
      return;
  }
  prefs.end();
  updateThresholdReadChr();
}

// Web UI writes 5 bytes: [thresholdId][uint32 LE value]
class ThresholdWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {
    String v = pChar->getValue();
    if (v.length() != 5) {
      Serial.println("Threshold write: bad length, expected 5 bytes");
      return;
    }
    uint8_t id = (uint8_t)v[0];
    uint32_t val;
    memcpy(&val, v.c_str() + 1, 4);
    setThresholdById(id, val);
  }
};

// The BLE stack stops advertising once a central connects to the
// server - restart it on disconnect so the config UI can reconnect,
// or a second device/tab can find it later.
class ConfigServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    Serial.println("Config UI connected");
  }
  void onDisconnect(BLEServer* server) {
    // Only resume advertising if we're still in Config Mode - otherwise
    // this would silently re-enable it after the mode was already
    // switched back to normal (e.g. web page closed after a BOOT hold).
    if (configMode) {
      Serial.println("Config UI disconnected, resuming advertising");
      BLEDevice::startAdvertising();
    } else {
      Serial.println("Config UI disconnected");
    }
  }
};

// ---------------------------------------------------------------
//  Serial command parser
// ---------------------------------------------------------------
void handleSerialCommands() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();

  if (line == "debug") {
    debugEnabled = true;
    Serial.println("Debug mode ENABLED");
    Serial.print("Print rate: every ");
    Serial.print(DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.print(" samples (~");
    Serial.print(SAMPLE_RATE / DEBUG_PRINT_EVERY_N_SAMPLES);
    Serial.println(" Hz)");
    return;
  }

  if (line == "exit") {
    debugEnabled = false;
    Serial.println("Debug mode DISABLED");
    return;
  }

  if (line == "status") {
    Serial.println("--------------------------------------------");
    Serial.println("BLE        : " + String(deviceConnected ? "Connected to ProtoBot" : "Disconnected"));
    Serial.println("Mode       : " + String(configMode ? "CONFIG (web UI)" : "NORMAL (driving ProtoBot)"));
    Serial.println("--------------------------------------------");
    return;
  }

  if (line == "config") {
    toggleConfigMode();
    return;
  }

  if (line.startsWith("set ")) {
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (secondSpace == -1) {
      Serial.println("Usage: set <fbsch1|fbsch3|leftch1|leftch2|rightch3> <value>");
      return;
    }
    String key = line.substring(firstSpace + 1, secondSpace);
    String valStr = line.substring(secondSpace + 1);
    valStr.trim();
    uint32_t val = (uint32_t)valStr.toInt();

    if (key == "fbsch1") setThresholdById(0, val);
    else if (key == "fbsch3") setThresholdById(1, val);
    else if (key == "leftch1") setThresholdById(2, val);
    else if (key == "leftch2") setThresholdById(3, val);
    else if (key == "rightch3") setThresholdById(4, val);
    else {
      Serial.print("Unknown key: ");
      Serial.println(key);
    }
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(line);
}

// ---------------------------------------------------------------
//  BLE status LED
// ---------------------------------------------------------------
void updateBLELed() {
  // While in Config Mode, enterConfigMode() has already painted every
  // NeoPixel blue - leave that display alone entirely (don't let the
  // single-index BLE_LED logic below touch it).
  if (configMode) return;

  uint32_t color;

  if (ledState == LED_RED) {
    color = pixel.Color(20, 0, 0);
  } else if (ledState == LED_GREEN) {
    color = pixel.Color(0, 20, 0);
  } else {
    unsigned long elapsed = millis() - lastCmdSentMs;
    if (elapsed < BLUE_LED_DURATION) {
      color = pixel.Color(0, 0, 30);
    } else {
      ledState = LED_GREEN;
      color = pixel.Color(0, 20, 0);
    }
  }

  if (color != lastPixel0Color) {
    lastPixel0Color = color;
    pixel.setPixelColor(BLE_LED, color);
    pixel.show();
  }
}

// ---------------------------------------------------------------
//  BLE client callbacks
// ---------------------------------------------------------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {}
  void onDisconnect(BLEClient* pclient) {
    deviceConnected = false;
    lastSentCmd = 255;
    ledState = LED_RED;
    pixelDirty = true;
    Serial.println("ProtoBot disconnected, re-scanning...");
    doScan = true;
  }
};

bool connectToProtoBot() {
  if (pClient) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  if (!pClient->connect(myDevice)) return false;

  BLERemoteService* svc = pClient->getService(joystickServiceUUID);
  if (!svc) { pClient->disconnect(); return false; }

  pJoystickChar = svc->getCharacteristic(joystickCharUUID);
  if (!pJoystickChar) { pClient->disconnect(); return false; }

  pSettingsChar = svc->getCharacteristic(settingsCharUUID);
  updateEyeColor(COLOR_STOP[0], COLOR_STOP[1], COLOR_STOP[2]);  // full settings packet (incl. speed_reduction) + starting color

  // Start centered/stopped, and reset the continuous-refresh state
  currentCommand    = 0;
  lastLoggedCmd     = 255;
  lastSentCmd       = 255;
  writeJoystick(JOY_CENTER, JOY_CENTER);
  lastJoystickSendMs = millis();

  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveServiceUUID() || !dev.isAdvertisingService(protobotAdvertisedUUID)) return;

    BLEDevice::getScan()->stop();
    if (myDevice) delete myDevice;
    myDevice  = new BLEAdvertisedDevice(dev);
    doConnect = true;
    doScan    = false;
    Serial.println("ProtoBot found: " + String(dev.getAddress().toString().c_str()));
  }
};

// ---------------------------------------------------------------
//  Config Mode enter/exit
// ---------------------------------------------------------------

void setAllPixels(uint32_t color) {
  for (int i = 0; i < pixel.numPixels(); i++) {
    pixel.setPixelColor(i, color);
  }
  pixel.show();
}

void enterConfigMode() {
  Serial.println("Config Mode ON  - pausing ProtoBot link, radio dedicated to config UI");
  doScan    = false;
  doConnect = false;
  BLEDevice::getScan()->stop();
  if (pClient && pClient->isConnected()) {
    pClient->disconnect();  // fires MyClientCallback::onDisconnect, which clears deviceConnected etc.
  }
  fbState = 0;
  fbConditionActive = false;
  lrActive = false;
  BLEDevice::getAdvertising()->start();

  setAllPixels(pixel.Color(0, 0, 40));  // all NeoPixels blue = Config Mode
}

void exitConfigMode() {
  Serial.println("Config Mode OFF - resuming ProtoBot link");
  BLEDevice::getAdvertising()->stop();
  doScan = true;

  setAllPixels(pixel.Color(0, 0, 0));  // clear the all-blue display...
  lastPixel0Color = 0xFFFFFFFF;        // ...then force BLE_LED to repaint on the next tick
  pixelDirty = true;                   // ...and force BATTERY_LED to repaint too
}

void toggleConfigMode() {
  configMode = !configMode;
  if (configMode) enterConfigMode();
  else exitConfigMode();
}

// Hold-to-toggle check for the BOOT button, called every loop()
// iteration. The toggle only fires on RELEASE, after confirming the
// button was held for BOOT_HOLD_MS - not partway through the hold.
// This matters beyond just avoiding repeat-fires: if this pin were
// ever misread as continuously low (wiring issue, no real button
// pressed), the old "fire mid-hold" version would flip into Config
// Mode once and then get stuck there forever, since a stuck-low pin
// never produces a release to arm the next toggle. Requiring an
// actual release means a stuck/floating pin just never toggles at
// all, so the device stays in its default Normal Mode instead of
// silently stranding itself in Config Mode.
void checkBootButton() {
  bool down = (digitalRead(BOOT_BUTTON_PIN) == LOW);  // active-low
  unsigned long now = millis();

  if (down && !bootButtonWasDown) {
    bootButtonDownAtMs = now;
  } else if (!down && bootButtonWasDown) {
    if (now - bootButtonDownAtMs >= BOOT_HOLD_MS) {
      toggleConfigMode();
    }
  }
  bootButtonWasDown = down;
}

// ---------------------------------------------------------------
//  Setup
// ---------------------------------------------------------------
void setup() {
  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.begin(BAUD_RATE);

  prefs.begin("thresholds", true);
  thrFbsCh1   = prefs.getUInt("fbsCh1thr", thrFbsCh1);
  thrFbsCh3   = prefs.getUInt("fbsCh3thr", thrFbsCh3);
  thrLeftCh1  = prefs.getUInt("leftCh1thr", thrLeftCh1);
  thrLeftCh2  = prefs.getUInt("leftCh2thr", thrLeftCh2);
  thrRightCh3 = prefs.getUInt("rightCh3thr", thrRightCh3);
  prefs.end();

  Serial.print("Loaded thrFbsCh1=");
  Serial.print(thrFbsCh1);
  Serial.print("  thrFbsCh3=");
  Serial.print(thrFbsCh3);
  Serial.print("  thrLeftCh1=");
  Serial.print(thrLeftCh1);
  Serial.print("  thrLeftCh2=");
  Serial.print(thrLeftCh2);
  Serial.print("  thrRightCh3=");
  Serial.println(thrRightCh3);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);

  BLEDevice::init("NPG-Lite-Controller");

  // --- Threshold config service (this board's PERIPHERAL/server role) ---
  pConfigServer = BLEDevice::createServer();
  pConfigServer->setCallbacks(new ConfigServerCallbacks());
  BLEService* pConfigService = pConfigServer->createService(configServiceUUID);

  pThreshWriteChr = pConfigService->createCharacteristic(
    thresholdWriteUUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pThreshWriteChr->setCallbacks(new ThresholdWriteCallbacks());

  pThreshReadChr = pConfigService->createCharacteristic(
    thresholdReadUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pThreshReadChr->addDescriptor(new BLE2902());

  pEmgLiveChr = pConfigService->createCharacteristic(
    emgLiveUUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pEmgLiveChr->addDescriptor(new BLE2902());
  pEmgLiveChr->setCallbacks(new EmgLiveReadCallbacks());

  pConfigService->start();
  updateThresholdReadChr();  // seed with the values just loaded from NVS above

  // Config service advertising is NOT started here - it only turns on
  // when Config Mode is entered (BOOT hold or serial "config"), so the
  // radio is fully dedicated to ProtoBot in normal operation.
  BLEAdvertising* pConfigAdvertising = BLEDevice::getAdvertising();
  pConfigAdvertising->addServiceUUID(configServiceUUID);
  pConfigAdvertising->setScanResponse(true);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);

  Serial.println("Scanning for any ProtoBot (CodeCell)...");
  doScan = true;
  scan->start(5, false);

  int initBattery = getCurrentBatteryPercentage();
  if (initBattery <= 20) {
    batteryColor = pixel.Color(20, 0, 0);
  } else if (initBattery <= 70) {
    batteryColor = pixel.Color(35, 7, 0);
  } else {
    batteryColor = pixel.Color(0, 20, 0);
  }
  pixel.setPixelColor(BATTERY_LED, batteryColor);
  pixel.show();

  printHelp();
}

void printHelp() {
  Serial.println("============================================");
  Serial.println("  NPG Lite -> ProtoBot direct control        ");
  Serial.println("  All 3 channels used as EMG.                ");
  Serial.println("  Ch1 (A0) = forward, Ch2 (A1) = right,       ");
  Serial.println("  Ch3 (A2) = left, both together = backward.  ");
  Serial.println("  status                     - show info     ");
  Serial.println("  config                     - toggle Config Mode");
  Serial.println("  (or hold BOOT for 1s - pauses ProtoBot link,");
  Serial.println("  dedicates radio to the web config page)     ");
  Serial.println("  debug / exit               - BCI debug log ");
  Serial.println("  (debug also pauses BLE scan for clean prints)");
  Serial.println("  set fbsch1   <n>  - Fwd/Back/Stop, ch1 leg ");
  Serial.println("  set fbsch3   <n>  - Fwd/Back/Stop, ch3 leg ");
  Serial.println("  set leftch1  <n>  - Left turn,      ch1 leg");
  Serial.println("  set leftch2  <n>  - Left turn,      ch2 leg");
  Serial.println("  set rightch3 <n>  - Right turn,     ch3 leg");
  Serial.println("  Thresholds can also be set live from the   ");
  Serial.println("  companion Web Bluetooth config page.       ");
  Serial.println("============================================");
}

// ---------------------------------------------------------------
//  Loop
// ---------------------------------------------------------------
unsigned long lastReconnect = 0;
const unsigned long RECONNECT_INTERVAL = 3000;

// ---- Forward/Backward/Stop debounce timing (state vars declared above) ----
static unsigned long fbLastChangeMs = 0;
const unsigned long FB_DEBOUNCE_MS = 300;  // min gap between cycle steps - tune to taste

static uint16_t emgLiveSampleCounter = 0;
const uint16_t EMG_LIVE_EVERY_N_SAMPLES = 10;  // 512 / 10 ~= 51.2 Hz notify rate

void loop() {
  static unsigned long lastMicros = micros();
  static long timer = 0;
  static int debugSampleCount = 0;

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;

  handleSerialCommands();
  checkBootButton();

  // --- Connect to ProtoBot when found (never while in Config Mode) ---
  if (doConnect && !configMode) {
    doConnect = false;
    if (connectToProtoBot()) {
      deviceConnected = true;
      ledState = LED_GREEN;
      pixelDirty = true;
      Serial.println("Connected to ProtoBot");

      // Fresh connection -> reset the fwd/back/stop cycle + turn overlay state
      fbState = 0;
      fbConditionActive = false;
      lrActive = false;
    } else {
      Serial.println("Connect failed, retrying...");
      doScan = true;
    }
  }

  // --- Re-scan while not connected ---
  // Skipped while debugEnabled: BLEScan::start() blocks the whole loop
  // for its duration, which starves the EMG sampling timer below and
  // makes debug prints stall/disappear. You don't need the robot
  // connected to read/calibrate EMG envelopes. Also skipped entirely
  // in Config Mode - scanning/connecting to ProtoBot is exactly what
  // Config Mode pauses so the radio is free for the config UI.
  if (!deviceConnected && doScan && !debugEnabled && !configMode) {
    unsigned long nowMs = millis();
    if (nowMs - lastReconnect >= RECONNECT_INTERVAL) {
      lastReconnect = nowMs;
      BLEDevice::getScan()->start(3, false);
    }
  }

  if (pixelDirty) {
    if (!configMode) {  // don't let a battery-color update overwrite the all-blue Config Mode display
      pixel.setPixelColor(BATTERY_LED, batteryColor);
      pixel.show();
    }
    pixelDirty = false;
  }
  updateBLELed();

  // --- Keep the active joystick command flowing at a steady rate ---
  if (deviceConnected && (millis() - lastJoystickSendMs >= JOYSTICK_REFRESH_MS)) {
    sendJoystickNow();
  }

  unsigned long currentMillis = millis();

  if (currentMillis - lastBatteryCheck >= BATTERY_CHECK_INTERVAL) {
    int currentBattery = getCurrentBatteryPercentage();
    if (currentBattery <= 20) {
      batteryColor = pixel.Color(20, 0, 0);
    } else if (currentBattery <= 70) {
      batteryColor = pixel.Color(35, 7, 0);
    } else {
      batteryColor = pixel.Color(0, 20, 0);
    }
    pixelDirty = true;
    lastBatteryCheck = currentMillis;
  }

  timer -= dt;
  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;

    int raw1 = analogRead(INPUT_PIN1);  // forward EMG
    int raw2 = analogRead(INPUT_PIN2);  // right EMG
    int raw3 = analogRead(INPUT_PIN3);  // left EMG
    batteryWinSum += analogRead(BATTERY_VOLTAGE_PIN);
    batteryWinCount++;

    float filtemg0 = emgfilters[0].process(filters[0].process(raw1));
    float filtemg1 = emgfilters[1].process(filters[1].process(raw2));
    float filtemg2 = emgfilters[2].process(filters[2].process(raw3));

    gEnv0 = Envelopefilter0.getEnvelope(abs(filtemg0));
    gEnv1 = Envelopefilter1.getEnvelope(abs(filtemg1));
    gEnv2 = Envelopefilter2.getEnvelope(abs(filtemg2));

    // Stream live envelope samples to a connected config-page client,
    // throttled down from the 512 Hz sample rate - ~51 Hz keeps the
    // slider backgrounds feeling responsive while still being light
    // enough not to flood the BLE stack. Only sent in Config Mode:
    // that's the whole point of the mode split - in normal mode the
    // radio is dedicated to ProtoBot, not shared with this stream.
    if (configMode && pConfigServer && pConfigServer->getConnectedCount() > 0) {
      if (++emgLiveSampleCounter >= EMG_LIVE_EVERY_N_SAMPLES) {
        emgLiveSampleCounter = 0;
        sendEmgLiveSample();
      }
    } else {
      emgLiveSampleCounter = 0;
    }

    if (debugEnabled) {
      if (++debugSampleCount >= DEBUG_PRINT_EVERY_N_SAMPLES) {
        debugSampleCount = 0;
        Serial.print("EMG_fwd: ");
        Serial.print(gEnv0, 2);
        Serial.print("  EMG_right: ");
        Serial.print(gEnv1, 2);
        Serial.print("  EMG_left: ");
        Serial.println(gEnv2, 2);
      }
    } else {
      debugSampleCount = 0;
    }

    if (deviceConnected) {
      unsigned long nowMs2 = millis();

      // ---- Forward/Backward/Stop: edge-triggered, debounced 3-state cycle ----
      // Each time all three EMG channels cross their high thresholds TOGETHER
      // (a fresh edge, not a held contraction), fbState advances:
      //   0 (stop) -> 4 (forward) -> 3 (backward) -> 0 (stop) -> ...
      bool fbCondNow = (gEnv0 > thrFbsCh1 && gEnv2 > thrFbsCh3);
      if (fbCondNow && !fbConditionActive && (nowMs2 - fbLastChangeMs >= FB_DEBOUNCE_MS)) {
        fbConditionActive = true;
        fbLastChangeMs = nowMs2;
        stage = true;

        if (fbState == 0) fbState = 4;        // stop -> forward
        else if (fbState == 4) fbState = 3;   // forward -> backward
        else fbState = 0;                     // backward -> stop

        if (!lrActive) applyCommand(fbState);  // don't override an active turn
      } else if (!fbCondNow) {
        fbConditionActive = false;             // rearm - next contraction is a new edge
      }

      // ---- Left / Right: independent overlay, works in any fb state ----
      // PRIORITY: Fwd/Back/Stop > Left > Right. Right turn's condition
      // is ch3 alone, which the FBS combo (ch1+ch3 together) always
      // also satisfies - so without this guard, every forward/backward
      // gesture also fired a spurious right-turn overlay on top of it.
      // Skipping left/right entirely while fbCondNow is true fixes
      // that: FBS gets to happen cleanly, and only once it's not
      // active do we even look at left/right (where left still takes
      // priority over right, same as before).
      if (!fbCondNow) {
        bool lrCondNow = false;
        if (gEnv0 > thrLeftCh1 && gEnv1 > thrLeftCh2) {
          lrCondNow = true;
          applyCommand(1);  // left
        } else if (gEnv2 > thrRightCh3) {
          lrCondNow = true;
          applyCommand(2);  // right
        }

        if (lrCondNow) {
          lrActive = true;
        } else if (lrActive) {
          lrActive = false;
          applyCommand(fbState);  // release turn -> resume whichever of {0,3,4} was active
        }
      } else if (lrActive) {
        // FBS just became the priority condition while a turn was
        // overlaying - release the turn cleanly instead of letting it
        // keep fighting the FBS command.
        lrActive = false;
        applyCommand(fbState);
      }
    }
  }
}
