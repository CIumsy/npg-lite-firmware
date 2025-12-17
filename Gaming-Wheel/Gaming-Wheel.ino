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

// Copyright (c) 2025 Krishnanshu Mittal - [krishnanshu@upsidedownlabs.tech]
// Copyright (c) 2025 Deepak Khatri - [deepak@upsidedownlabs.tech]
// Copyright (c) 2025 Upside Down Labs - [contact@upsidedownlabs.tech]

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

// Core includes
#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <BleCombo.h>

// ── MPU6050 Wheel Rotation Includes ──
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ══════════════════════════════════════════════════════════════════════════════
// ─── 🎮 KEY ENABLE/DISABLE (Comment out to disable) ───
// ══════════════════════════════════════════════════════════════════════════════
#define ENABLE_MPU_KEY_A          // Left wheel rotation
#define ENABLE_MPU_KEY_D          // Right wheel rotation

#define ENABLE_EMG_KEY_W          // env1 (INDEPENDENT)
#define ENABLE_EMG_KEY_S          // env2 (INDEPENDENT)

#define ENABLE_BOOT_BUTTON        // Boot button (Pin 9) -> H key
// ══════════════════════════════════════════════════════════════════════════════

// ─── 🎮 KEY MAPPING CONFIGURATION ───
#define MPU_KEY_A           'a'              // Left wheel rotation
#define MPU_KEY_D           'd'              // Right wheel rotation

#define EMG_KEY_W           'w'              // env1 (INDEPENDENT)
#define EMG_KEY_S           's'              // env2 (INDEPENDENT)

#define BOOT_BUTTON_KEY     'h'              // Boot button key
// ══════════════════════════════════════════════════════════════════════════════

// ── MPU6050 SETTINGS ───
#define VIBRATION_PIN       7                // Vibration motor for calibration feedback
#define BOOT_BUTTON_PIN     9                // Boot button pin

// ── 2-LEVEL THRESHOLD TUNING ──
#define THRESHOLD_1         2.0              // Light tilt (burst mode)
#define THRESHOLD_2         8.0              // Heavy tilt (continuous hold)

#define BURST_ON_MS         100              // Threshold 1: 100ms on, 100ms off
#define BURST_OFF_MS        100

#define PULSE_ON_MS         200              // Threshold 2: 200ms on, 100ms off
#define PULSE_OFF_MS        100

#define SENSORS_RADS_TO_DPS 57.2957795f      // Conversion factor

// ─── MPU6050 Variables ───
Adafruit_MPU6050 mpu;
float neutralRoll = 0;
float smoothedRoll = 0;
float lastSmoothedRoll = 0;
bool isMPUCalibrated = false;
int rollDirection = 1;
bool axisCalibrated = false;

enum CalibrationState {
  CAL_IDLE,
  CAL_INIT_WAIT,
  CAL_LEFT_VIBRATE,
  CAL_LEFT_WAIT,
  CAL_RIGHT_VIBRATE,
  CAL_RIGHT_WAIT,
  CAL_NEUTRAL_SAMPLE,
  CAL_COMPLETE
};

CalibrationState calState = CAL_IDLE;
unsigned long calStateStartTime = 0;
float calStartRoll = 0, calEndRoll = 0;
int neutralSampleCount = 0;
float neutralRollSum = 0;

// MPU key state tracking with pulse timing
bool mpuKeyAPressed = false;
bool mpuKeyDPressed = false;
unsigned long mpuKeyAPressTime = 0;
unsigned long mpuKeyDPressTime = 0;

// Boot button state tracking
bool bootButtonPressed = false;
bool bootKeyPressed = false;

// ─── EMG Signal processing config ───
#define SAMPLE_RATE   512
#define INPUT_PIN1    A0    // EMG channel 1
#define INPUT_PIN2    A1    // EMG channel 2

class NotchFilter {
private:
  struct BiquadState { float z1 = 0, z2 = 0; };
  BiquadState state0;
  BiquadState state1;

public:
  float process(float input) {
    float output = input;
    float x0 = output - (-1.58696045f * state0.z1) - (0.96505858f * state0.z2);
    output = 0.96588529f * x0 + -1.57986211f * state0.z1 + 0.96588529f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;

    float x1 = output - (-1.62761184f * state1.z1) - (0.96671306f * state1.z2);
    output = 1.00000000f * x1 + -1.63566226f * state1.z1 + 1.00000000f * state1.z2;
    state1.z2 = state1.z1;
    state1.z1 = x1;

    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
    state1.z1 = state1.z2 = 0;
  }
} filters[2];  // Only 2 notch filters

class EMGFilter {
private:
  struct BiquadState { float z1 = 0, z2 = 0; };
  BiquadState state0;

public:
  float process(float input) {
    float output = input;
    float x0 = output - (-0.85080258f * state0.z1) - (0.30256882f * state0.z2);
    output = 0.53834285f * x0 + -1.07668570f * state0.z1 + 0.53834285f * state0.z2;
    state0.z2 = state0.z1;
    state0.z1 = x0;
    return output;
  }

  void reset() {
    state0.z1 = state0.z2 = 0;
  }
} emgfilters[2];  // Only 2 EMG high-pass filters

class EnvelopeFilter {
private:
  std::vector<double> buf;
  double sum = 0.0;
  int idx = 0;
  const int N;
public:
  EnvelopeFilter(int n) : N(n) { buf.resize(N, 0.0); }
  double getEnvelope(double v) {
    sum -= buf[idx];
    sum += v;
    buf[idx] = v;
    idx = (idx + 1) % N;
    return sum / N;
  }
} env1(16), env2(16);  // Only 2 envelope filters

bool emgKeyWPressed = false;
bool emgKeySPressed = false;

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
  if (voltage <= voltageLUT[0]) return 0;
  if (voltage >= voltageLUT[lutSize - 1]) return 100;
  int i = 0;
  while (voltage > voltageLUT[i + 1]) i++;
  float v1 = voltageLUT[i], v2 = voltageLUT[i + 1];
  int p1 = percentLUT[i], p2 = percentLUT[i + 1];
  return p1 + (voltage - v1) * (p2 - p1) / (v2 - v1);
}

int getCurrentBatteryPercentage() {
  int analogValue = analogRead(A6);
  float voltage = (analogValue / 1000.0) * 2;
  voltage += 0.022;
  float percentage = interpolatePercentage(voltage);
  return (int)percentage;
}

void startVibration() {
  digitalWrite(VIBRATION_PIN, HIGH);
}

void stopVibration() {
  digitalWrite(VIBRATION_PIN, LOW);
}

void updateCalibrationStateMachine(unsigned long nowMs) {
  if (calState == CAL_IDLE || calState == CAL_COMPLETE) return;

  unsigned long elapsed = nowMs - calStateStartTime;
  sensors_event_t a, g, temp;

  switch (calState) {
    case CAL_INIT_WAIT:
      if (elapsed >= 3000) {
        calState = CAL_LEFT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
        mpu.getEvent(&a, &g, &temp);
        calStartRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      }
      break;

    case CAL_LEFT_VIBRATE:
      if (elapsed >= 3000) {
        stopVibration();
        calState = CAL_LEFT_WAIT;
        calStateStartTime = nowMs;
        mpu.getEvent(&a, &g, &temp);
        calEndRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
        rollDirection = ((calEndRoll - calStartRoll) > 0) ? -1 : 1;
      }
      break;

    case CAL_LEFT_WAIT:
      if (elapsed >= 3000) {
        calState = CAL_RIGHT_VIBRATE;
        calStateStartTime = nowMs;
        startVibration();
      }
      break;

    case CAL_RIGHT_VIBRATE:
      if (elapsed >= 3000) {
        stopVibration();
        calState = CAL_RIGHT_WAIT;
        calStateStartTime = nowMs;
        axisCalibrated = true;
      }
      break;

    case CAL_RIGHT_WAIT:
      if (elapsed >= 2000) {
        calState = CAL_NEUTRAL_SAMPLE;
        calStateStartTime = nowMs;
        neutralSampleCount = 0;
        neutralRollSum = 0;
      }
      break;

    case CAL_NEUTRAL_SAMPLE:
      if (neutralSampleCount < 100) {
        mpu.getEvent(&a, &g, &temp);
        float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
        neutralRollSum += roll;
        neutralSampleCount++;
      } else {
        neutralRoll = neutralRollSum / 100;
        smoothedRoll = neutralRoll;
        lastSmoothedRoll = neutralRoll;
        isMPUCalibrated = true;
        calState = CAL_COMPLETE;
      }
      break;

    default:
      break;
  }
}

// ─── 2-LEVEL THRESHOLD BURST SYSTEM ───
void updateMPUKeys(unsigned long nowMs) {
  if (!isMPUCalibrated || !axisCalibrated) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float currentRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

  const float SMOOTHING = 0.7f;
  smoothedRoll = SMOOTHING * smoothedRoll + (1.0f - SMOOTHING) * currentRoll;

  float deltaRoll = (smoothedRoll - neutralRoll) * rollDirection;

#ifdef ENABLE_MPU_KEY_A
  // ─── LEFT STEERING ───
  if (deltaRoll < -THRESHOLD_1) {
    if (!mpuKeyAPressed) {
      Keyboard.press(MPU_KEY_A);
      mpuKeyAPressed = true;
      mpuKeyAPressTime = nowMs;
    }

    // Determine pulse timing based on threshold level
    unsigned long onTime, offTime;
    if (deltaRoll < -THRESHOLD_2) {
      // Heavy tilt: continuous hold
      onTime = 1000;
      offTime = 50;
    } else {
      // Light tilt: burst mode (100ms on, 100ms off)
      onTime = BURST_ON_MS;
      offTime = BURST_OFF_MS;
    }

    // Handle pulse timing
    unsigned long elapsed = nowMs - mpuKeyAPressTime;
    bool shouldBePressed = (elapsed % (onTime + offTime)) < onTime;

    if (shouldBePressed && !mpuKeyAPressed) {
      Keyboard.press(MPU_KEY_A);
      mpuKeyAPressed = true;
    } else if (!shouldBePressed && mpuKeyAPressed) {
      Keyboard.release(MPU_KEY_A);
      mpuKeyAPressed = false;
    }
  } else {
    // Release when back to center
    if (mpuKeyAPressed) {
      Keyboard.release(MPU_KEY_A);
      mpuKeyAPressed = false;
    }
  }
#endif

#ifdef ENABLE_MPU_KEY_D
  // ─── RIGHT STEERING ───
  if (deltaRoll > THRESHOLD_1) {
    if (!mpuKeyDPressed) {
      Keyboard.press(MPU_KEY_D);
      mpuKeyDPressed = true;
      mpuKeyDPressTime = nowMs;
    }

    // Determine pulse timing based on threshold level
    unsigned long onTime, offTime;
    if (deltaRoll > THRESHOLD_2) {
      // Heavy tilt: continuous hold
      onTime = 1000;
      offTime = 50;
    } else {
      // Light tilt: burst mode
      onTime = BURST_ON_MS;
      offTime = BURST_OFF_MS;
    }

    // Handle pulse timing
    unsigned long elapsed = nowMs - mpuKeyDPressTime;
    bool shouldBePressed = (elapsed % (onTime + offTime)) < onTime;

    if (shouldBePressed && !mpuKeyDPressed) {
      Keyboard.press(MPU_KEY_D);
      mpuKeyDPressed = true;
    } else if (!shouldBePressed && mpuKeyDPressed) {
      Keyboard.release(MPU_KEY_D);
      mpuKeyDPressed = false;
    }
  } else {
    // Release when back to center
    if (mpuKeyDPressed) {
      Keyboard.release(MPU_KEY_D);
      mpuKeyDPressed = false;
    }
  }
#endif

  lastSmoothedRoll = smoothedRoll;
}

// ─── BOOT BUTTON HANDLER ───
void updateBootButton() {
#ifdef ENABLE_BOOT_BUTTON
  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);
  
  // Button is active LOW (pressed = LOW)
  if (currentButtonState == LOW && !bootButtonPressed) {
    // Button just pressed
    bootButtonPressed = true;
    if (!bootKeyPressed) {
      Keyboard.press(BOOT_BUTTON_KEY);
      bootKeyPressed = true;
    }
  } else if (currentButtonState == HIGH && bootButtonPressed) {
    // Button just released
    bootButtonPressed = false;
    if (bootKeyPressed) {
      Keyboard.release(BOOT_BUTTON_KEY);
      bootKeyPressed = false;
    }
  }
#endif
}

void setup() {
  Wire.begin(22, 23);

  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(A6, INPUT);

  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, HIGH);
  delay(200);
  digitalWrite(VIBRATION_PIN, LOW);

#ifdef ENABLE_BOOT_BUTTON
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);  // Internal pullup for boot button
#endif

  Keyboard.begin();
  Mouse.begin();

  if (!mpu.begin()) {
    isMPUCalibrated = false;
    axisCalibrated = false;
    calState = CAL_COMPLETE;
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    calState = CAL_INIT_WAIT;
    calStateStartTime = millis();
  }
}

void loop() {
  static unsigned long lastMicros = micros();

  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;

  unsigned long nowMs = millis();

  updateCalibrationStateMachine(nowMs);
  updateMPUKeys(nowMs);
  updateBootButton();

  if (timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;

    // Read only 2 EMG channels
    int raw1 = analogRead(INPUT_PIN1);
    int raw2 = analogRead(INPUT_PIN2);

    // Filter and envelope extraction for 2 channels
    float filtemg1 = emgfilters[0].process(filters[0].process(raw1));
    float filtemg2 = emgfilters[1].process(filters[1].process(raw2));

    float envelope1 = env1.getEnvelope(fabs(filtemg1));
    float envelope2 = env2.getEnvelope(fabs(filtemg2));

#ifdef ENABLE_EMG_KEY_W
    // EMG_KEY_W (env1): INDEPENDENT - triggers when env1 > 150
    bool conditionW = (envelope1 > 40);
    if (conditionW) {
      if (!emgKeyWPressed) {
        Keyboard.press(EMG_KEY_W);
        emgKeyWPressed = true;
      }
    } else {
      if (emgKeyWPressed) {
        Keyboard.release(EMG_KEY_W);
        emgKeyWPressed = false;
      }
    }
#endif

#ifdef ENABLE_EMG_KEY_S
    // EMG_KEY_S (env2): INDEPENDENT - triggers when env2 > 150
    bool conditionS = (envelope2 > 40);
    if (conditionS) {
      if (!emgKeySPressed) {
        Keyboard.press(EMG_KEY_S);
        emgKeySPressed = true;
      }
    } else {
      if (emgKeySPressed) {
        Keyboard.release(EMG_KEY_S);
        emgKeySPressed = false;
      }
    }
#endif
  }
}
