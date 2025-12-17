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
//
// Copyright (c) 2025 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2025 Deepak Khatri - deepak@upsidedownlabs.tech
// Copyright (c) 2025 Upside Down Labs - contact@upsidedownlabs.tech
//
// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

/*
FFT routines based on Espressif's ESP-DSP examples:
• Initialization (dsps_fft2r_init_fc32) from:
https://github.com/espressif/esp-dsp/tree/master/examples/basic_math
(examples/basic_math/main/dsps_math_main.c)
• Two-real FFT processing
(dsps_fft2r_fc32, dsps_bit_rev_fc32, dsps_cplx2reC_fc32)
from: https://github.com/espressif/esp-dsp/tree/master/examples/fft
(examples/fft/main/dsps_fft_main.c)
*/
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_dsp.h"
#define PIXEL_PIN         15
#define PIXEL_COUNT       6

// Manual control configuration
#define MANUAL_CONTROL 0 // OFF when 0, ON when 1 (default: EEG+EOG mode)

// Control mode runtime variable
bool manualControlMode = MANUAL_CONTROL; // Runtime mode toggle

// ----------------- BCI CONFIGURATION -----------------
#define SAMPLE_RATE 512 // samples per second
#define FFT_SIZE 512 // must be a power of two
#define INPUT_PIN A0

// Onboard Neopixel at PIXEL_PIN
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// EEG bands (Hz)
#define DELTA_LOW 0.5f
#define DELTA_HIGH 4.0f
#define THETA_LOW 4.0f
#define THETA_HIGH 8.0f
#define ALPHA_LOW 8.0f
#define ALPHA_HIGH 13.0f
#define BETA_LOW 13.0f
#define BETA_HIGH 30.0f
#define GAMMA_LOW 30.0f
#define GAMMA_HIGH 45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS 1e-7f

// Envelope detection
#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)

// Blink detection parameters
const unsigned long BLINK_DEBOUNCE_MS = 300;
const unsigned long DOUBLE_BLINK_MS = 600;
float BlinkThreshold = 50.0;

// Focus detection threshold (fixed value instead of calibration)
float betaThreshold = 12.0; // Adjust this value based on your testing

// Focus detection debouncing
unsigned long lastFocusTime = 0;
const unsigned long focusDebounceMs = 2000; // 1 second debounce for focus events

// NeoPixel animation variables
unsigned long lastledBlinkUpdate = 0;
const unsigned long ledblinkInterval = 30; // Smooth fade update interval
float ledblinkBrightness = 0.0;
float ledblinkDirection = 1.0; // 1 for increasing, -1 for decreasing
const float ledblinkSpeed = 0.05; // Speed of fade

// Transition variables for enabling animation
bool isTransitioning = false;
unsigned long transitionStartTime = 0;
const unsigned long transitionDuration = 500; // 500ms for red to green transition

// ----------------- ORIGINAL RELAY CODE PINS -----------------
const int ledPin = 7;
const int bootButtonPin = 9; // Boot button on NPG Lite (ESP32-C6)

// Boot button mode switching (hold for 3 seconds to toggle mode)
unsigned long bootButtonPressStartTime = 0;
bool bootButtonHeld = false;
bool modeJustSwitched = false;
const unsigned long MODE_SWITCH_HOLD_TIME = 3000; // 3 seconds to switch mode

// Boot button double-click detection (for manual mode)
unsigned long lastBootButtonPress = 0;
unsigned long firstBootButtonPress = 0;
int bootButtonClickCount = 0;
const unsigned long BOOT_BUTTON_DEBOUNCE_MS = 50;
const unsigned long DOUBLE_CLICK_MS = 400; // Time window for double click

// ----------------- BCI VARIABLES -----------------
// Buffers & types
float inputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE/2];
__attribute__((aligned(16))) float y_cf[FFT_SIZE * 2];
float *y1_cf = &y_cf[0];

// Envelope detection
float envelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int envelopeIndex = 0;
float envelopeSum = 0;
float currentEEGEnvelope = 0;

// Bandpower structure
typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

BandpowerResults smoothedPowers = {0};

// Blink detection variables
unsigned long lastBlinkTime = 0;
unsigned long firstBlinkTime = 0;
int blinkCount = 0;

// ----------------- ORIGINAL RELAY VARIABLES -----------------
uint8_t currentChannel = 0;  // Start with channel 0 (will become 1 on first double blink)
bool ledState[6] = {false, false, false, false, false, false}; // Track enabled state of each channel
uint32_t buttonState;
int lastButtonState = HIGH; // Boot button is active LOW (pulled up)
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// BLE variables
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic_1 = NULL;
BLECharacteristic* pCharacteristic_2 = NULL;
BLEDescriptor *pDescr_1;
BLE2902 *pBLE2902_1;
BLE2902 *pBLE2902_2;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_1 "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// ----------------- FILTER FUNCTIONS -----------------
// High-Pass Butterworth IIR digital filter
float highpass(float input) {
  float output = input;
  static float z1, z2;
  float x = output - -1.91327599*z1 - 0.91688335*z2;
  output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
  z2 = z1;
  z1 = x;
  return output;
}

// Band-Stop Butterworth IIR digital filter (50Hz notch)
float Notch(float input) {
  float output = input;
  
  static float z1_1, z2_1;
  float x = output - -1.58696045*z1_1 - 0.96505858*z2_1;
  output = 0.96588529*x + -1.57986211*z1_1 + 0.96588529*z2_1;
  z2_1 = z1_1;
  z1_1 = x;
  
  static float z1_2, z2_2;
  x = output - -1.62761184*z1_2 - 0.96671306*z2_2;
  output = 1.00000000*x + -1.63566226*z1_2 + 1.00000000*z2_2;
  z2_2 = z1_2;
  z1_2 = x;
  
  return output;
}

// Low-Pass Butterworth IIR digital filter
float EEGFilter(float input) {
  float output = input;
  static float z1, z2;
  float x = output - -1.24200128*z1 - 0.45885207*z2;
  output = 0.05421270*x + 0.10842539*z1 + 0.05421270*z2;
  z2 = z1;
  z1 = x;
  return output;
}

// ----------------- ENVELOPE FUNCTION -----------------
float updateEEGEnvelope(float sample) {
  float absSample = fabs(sample);
  envelopeSum -= envelopeBuffer[envelopeIndex];
  envelopeSum += absSample;
  envelopeBuffer[envelopeIndex] = absSample;
  envelopeIndex = (envelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return envelopeSum / ENVELOPE_WINDOW_SIZE;
}

// ----------------- BANDPOWER & SMOOTHING -----------------
BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = {0};
  
  for(int i = 1; i < halfSize; i++) {
    float freq = i * binRes;
    if(freq >= DELTA_LOW && freq < DELTA_HIGH) r.delta += ps[i];
    else if(freq >= THETA_LOW && freq < THETA_HIGH) r.theta += ps[i];
    else if(freq >= ALPHA_LOW && freq < ALPHA_HIGH) r.alpha += ps[i];
    else if(freq >= BETA_LOW && freq < BETA_HIGH) r.beta += ps[i];
    else if(freq >= GAMMA_LOW && freq < GAMMA_HIGH) r.gamma += ps[i];
    r.total += ps[i];
  }
  return r;
}

void smoothBandpower(BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR*raw->delta + (1-SMOOTHING_FACTOR)*s->delta;
  s->theta = SMOOTHING_FACTOR*raw->theta + (1-SMOOTHING_FACTOR)*s->theta;
  s->alpha = SMOOTHING_FACTOR*raw->alpha + (1-SMOOTHING_FACTOR)*s->alpha;
  s->beta = SMOOTHING_FACTOR*raw->beta + (1-SMOOTHING_FACTOR)*s->beta;
  s->gamma = SMOOTHING_FACTOR*raw->gamma + (1-SMOOTHING_FACTOR)*s->gamma;
  s->total = SMOOTHING_FACTOR*raw->total + (1-SMOOTHING_FACTOR)*s->total;
}

// ----------------- NEOPIXEL UPDATE FUNCTION -----------------
void updateNeoPixels() {
  pixels.clear();
  
  // Show mode indicator - if mode just switched, flash all LEDs briefly
  if (modeJustSwitched) {
    static unsigned long modeSwitchTime = 0;
    static bool modeIndicatorShown = false;
    
    if (!modeIndicatorShown) {
      modeSwitchTime = millis();
      modeIndicatorShown = true;
    }
    
    unsigned long elapsed = millis() - modeSwitchTime;
    if (elapsed < 500) {
      // Flash indicator color based on mode
      uint8_t flash = (elapsed / 100) % 2 ? 30 : 0; // Blink 5 times
      if (manualControlMode) {
        // Manual mode: Flash yellow
        for (int i = 0; i < 6; i++) {
          pixels.setPixelColor(i, pixels.Color(flash, flash, 0));
        }
      } else {
        // EEG+EOG mode: Flash cyan
        for (int i = 0; i < 6; i++) {
          pixels.setPixelColor(i, pixels.Color(0, flash, flash));
        }
      }
      pixels.show();
      return;
    } else {
      modeJustSwitched = false;
      modeIndicatorShown = false;
    }
  }
  
  // Update all LEDs based on their state
  for (int i = 0; i < 6; i++) {
    if (i == currentChannel - 1 && currentChannel > 0) {
      // Current/hovering LED
      if (isTransitioning) {
        // Transitioning from red to green (or red to off if disabling)
        unsigned long elapsed = millis() - transitionStartTime;
        float progress = min(1.0f, (float)elapsed / transitionDuration);
        
        // Smooth easing function (ease-out)
        progress = 1.0 - pow(1.0 - progress, 3);
        
        if (ledState[i]) {
          // Enabling: Interpolate from red to green
          uint8_t r = (uint8_t)(40 * (1.0 - progress)); 
          uint8_t g = (uint8_t)(40 * progress);
          pixels.setPixelColor(i, pixels.Color(r, g, 0));
        } else {
          // Disabling: Fade out red
          uint8_t r = (uint8_t)(40 * (1.0 - progress));
          pixels.setPixelColor(i, pixels.Color(r, 0, 0));
        }
        
        if (progress >= 1.0) {
          isTransitioning = false;
        }
      } else if (ledState[i]) {
        // Already enabled, show green 
        pixels.setPixelColor(i, pixels.Color(0, 40, 0));
      } else {
        // Hovering, blink red with fade (10-40)
        uint8_t brightness = (uint8_t)(10 + ledblinkBrightness * 30);
        pixels.setPixelColor(i, pixels.Color(brightness, 0, 0));
      }
    } else if (ledState[i]) {
      // Previously enabled LED, show blue (static 10)
      pixels.setPixelColor(i, pixels.Color(0, 0, 10));
    }
    // else: LED is off (already cleared)
  }
  
  pixels.show();
}

// ----------------- DSP FFT SETUP -----------------
void initFFT() {
  esp_err_t err = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if(err != ESP_OK){
    Serial.println("FFT init failed");
    while(1) delay(10);
  }
}

// ----------------- FFT + BANDPOWER PROCESSING -----------------
void processFFT() {
  // Pack real→complex: real=inputBuffer, imag=0
  for(int i = 0; i < FFT_SIZE; i++) {
    y1_cf[i*2 + 0] = inputBuffer[i];
    y1_cf[i*2 + 1] = 0;
  }
  
  // Two-real FFT
  dsps_fft2r_fc32(y1_cf, FFT_SIZE);
  dsps_bit_rev_fc32(y1_cf, FFT_SIZE);
  dsps_cplx2reC_fc32(y1_cf, FFT_SIZE);
  
  // Power spectrum
  int half = FFT_SIZE/2;
  for(int i = 0; i < half; i++) {
    float re = y1_cf[i*2 + 0];
    float im = y1_cf[i*2 + 1];
    powerSpectrum[i] = re*re + im*im;
  }
  
  float binRes = float(SAMPLE_RATE)/FFT_SIZE;
  
  // Bandpower & smoothing
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  
  float T = smoothedPowers.total + EPS;
  float betaPct = (smoothedPowers.beta / T) * 100.0;
  Serial.println(betaPct);
  
  // Check for focus (beta) event - Toggle current channel (only in EEG+EOG mode)
  if (!manualControlMode) {
    unsigned long currentTime = millis();
    if (betaPct > betaThreshold && currentChannel > 0 && currentChannel <= 6) {
      // Debounce focus detection
      if (currentTime - lastFocusTime >= focusDebounceMs) {
        lastFocusTime = currentTime;
        
        // Toggle the LED state
        ledState[currentChannel - 1] = !ledState[currentChannel - 1];
        
        Serial.print("Focus detected! Toggling channel: ");
        Serial.print(currentChannel);
        Serial.print(" to ");
        Serial.println(ledState[currentChannel - 1] ? "ON" : "OFF");
        
        // Start transition animation
        isTransitioning = true;
        transitionStartTime = currentTime;
        
        if (deviceConnected) {
          pCharacteristic_1->setValue(&currentChannel, 1);
          pCharacteristic_1->notify();
        }
      }
    }
  }
}

// ----------------- BOOT BUTTON TOGGLE FUNCTION -----------------
void toggleCurrentChannel() {
  if (currentChannel > 0 && currentChannel <= 6) {
    // Toggle the LED state
    ledState[currentChannel - 1] = !ledState[currentChannel - 1];
    
    Serial.print("Boot button toggle! Channel: ");
    Serial.print(currentChannel);
    Serial.print(" to ");
    Serial.println(ledState[currentChannel - 1] ? "ON" : "OFF");
    
    // Start transition animation
    isTransitioning = true;
    transitionStartTime = millis();
    
    if (deviceConnected) {
      pCharacteristic_1->setValue(&currentChannel, 1);
      pCharacteristic_1->notify();
    }
  }0
}

// ----------------- BLE CALLBACK -----------------
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  pinMode(bootButtonPin, INPUT_PULLUP); // Boot button always enabled for mode switching
  
  digitalWrite(ledPin, LOW);
  
  // Create the BLE Device
  BLEDevice::init("ESP32");
  
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create BLE Characteristics
  pCharacteristic_1 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_1,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );                   

  pCharacteristic_2 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_2,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |                      
                      BLECharacteristic::PROPERTY_NOTIFY
                    );  

  // Create a BLE Descriptor  
  pDescr_1 = new BLEDescriptor((uint16_t)0x2901);
  pDescr_1->setValue("A very interesting variable");
  pCharacteristic_1->addDescriptor(pDescr_1);

  // Add the BLE2902 Descriptor
  pBLE2902_1 = new BLE2902();
  pBLE2902_1->setNotifications(true);                 
  pCharacteristic_1->addDescriptor(pBLE2902_1);

  pBLE2902_2 = new BLE2902();
  pBLE2902_2->setNotifications(true);
  pCharacteristic_2->addDescriptor(pBLE2902_2);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("Waiting a client connection to notify...");
  
  // Initialize FFT
  initFFT();
  
  // Initialize NeoPixels - all off at start
  pixels.begin();
  pixels.clear();
  pixels.show();
}

// ----------------- MAIN LOOP -----------------
void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = micros();
  unsigned long now = micros(), dt = now - lastMicros;
  lastMicros = now;
  static long timer = 0;
  timer -= dt;
  
  // EEG sampling and processing
  if(timer <= 0) {
    timer += 1000000L / SAMPLE_RATE;
    
    int raw = analogRead(INPUT_PIN);
    float filt = EEGFilter(Notch(raw));
    float filtered = highpass(filt);
    currentEEGEnvelope = updateEEGEnvelope(filtered);
    inputBuffer[idx++] = filt;
    
    // Process FFT when buffer is full
    if(idx >= FFT_SIZE) {
      processFFT();
      idx = 0;
    }
  }

  
  // Handle double blink detection for channel switching (only in EEG+EOG mode)
  unsigned long nowMs = millis();
  
  if (!manualControlMode && currentEEGEnvelope > BlinkThreshold && (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {
    lastBlinkTime = nowMs;
    
    if (blinkCount == 0) {
      // First blink of the pair
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_MS) {
      // Second blink in time → switch to next channel (NO NOTIFICATION)
      if(currentChannel == 6) currentChannel = 1;
      else currentChannel++;

      Serial.print("Double blink detected! Switched to channel: ");
      Serial.println(currentChannel);
      // NO BLE notification sent here - only switching channel
      
      blinkCount = 0; // Reset for next pair
    }
    else {
      // Either too late or extra blink → restart sequence
      firstBlinkTime = nowMs;
      blinkCount = 1;
    }
  }
  
  // Timeout: if we never got the second blink in time, reset
  if (!manualControlMode && blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_MS) {
    blinkCount = 0;
  }
  
  // Always handle boot button - for mode switching (3s hold) and manual control
  int buttonReading = digitalRead(bootButtonPin);
  
  // Detect button press (HIGH to LOW transition)
  if (buttonReading == LOW && lastButtonState == HIGH) {
    // Button just pressed
    bootButtonPressStartTime = nowMs;
    bootButtonHeld = true;
  }
  
  // Check if button is being held for mode switch
  if (buttonReading == LOW && bootButtonHeld && !modeJustSwitched) {
    unsigned long holdDuration = nowMs - bootButtonPressStartTime;
    
    if (holdDuration >= MODE_SWITCH_HOLD_TIME) {
      // Toggle mode
      manualControlMode = !manualControlMode;
      modeJustSwitched = true;
      bootButtonHeld = false;
      
      Serial.print("Mode switched to: ");
      Serial.println(manualControlMode ? "MANUAL" : "EEG+EOG");
      
      // Reset any ongoing operations
      blinkCount = 0;
      bootButtonClickCount = 0;
    }
  }
  
  // Detect button release (LOW to HIGH transition)
  if (buttonReading == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = nowMs - bootButtonPressStartTime;
    bootButtonHeld = false;
    
    // Only process clicks if in manual mode and not a mode switch
    if (manualControlMode && pressDuration < MODE_SWITCH_HOLD_TIME) {
      unsigned long timeSinceLastPress = nowMs - lastBootButtonPress;
      
      // Check if this could be part of a double-click
      if (timeSinceLastPress < DOUBLE_CLICK_MS && bootButtonClickCount == 1) {
        // Double click detected - switch channel
        if(currentChannel == 6) currentChannel = 1;
        else currentChannel++;
        
        Serial.print("Boot button double-click! Switched to channel: ");
        Serial.println(currentChannel);
        
        bootButtonClickCount = 0; // Reset click count
      } else {
        // First click detected
        bootButtonClickCount = 1;
        firstBootButtonPress = nowMs;
      }
      
      lastBootButtonPress = nowMs;
    }
  }
  
  // Timeout for single click - if no second click within the window (manual mode only)
  if (manualControlMode && bootButtonClickCount == 1 && (nowMs - firstBootButtonPress) > DOUBLE_CLICK_MS) {
    // Single click confirmed - toggle current channel
    toggleCurrentChannel();
    bootButtonClickCount = 0; // Reset click count
  }
  
  lastButtonState = buttonReading;
  
  // Update NeoPixel blink animation
  if (nowMs - lastledBlinkUpdate >= ledblinkInterval) {
    lastledBlinkUpdate = nowMs;
    
    // Update blink brightness with smooth fade
    ledblinkBrightness += ledblinkDirection * ledblinkSpeed;
    
    if (ledblinkBrightness >= 1.0) {
      ledblinkBrightness = 1.0;
      ledblinkDirection = -1.0;
    } else if (ledblinkBrightness <= 0.0) {
      ledblinkBrightness = 0.0;
      ledblinkDirection = 1.0;
    }
  }
  
  // Update all NeoPixels
  updateNeoPixels();
  
  // Handle BLE connection state
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}
