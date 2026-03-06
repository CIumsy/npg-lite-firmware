# NPG Lite Wheelchair-Simulator Firmware

A simple open-source firmware for **Neuro PlayGround Lite (NPG Lite)** by Upside Down Labs. This project lets you control a computer (or game) using your **eye movements** and **jaw clenches**!

---

## What Does It Do?
- Reads bio-potential signals from your face (eyes and jaw) using a single analog input (A0) on Neuro Playground Lite
- Detects **left/right eye movement** and **jaw clench** (single/double)
- Sends keypresses (a/d/w/s) over Bluetooth as a BLE keyboard
- Shows BLE connection status with onboard NeoPixel LEDs

---

## Hardware Requirements
- **NPG Lite(Any pack)**
- **USB-C cable** 
- **3 Gel Electrodes**
- **3 BioAmp Snap Cables**
- **Nuprep Skin Preparation Gel**
- **Alcohol Swabs**
- **A Windows Laptop/Desktop**

---

## Software Requirements
- Arduino IDE (with ESP32 3.2.0 board version installed)
- Required libraries:
  - `BleCombo` (for BLE HID keyboard)
  - `Adafruit_NeoPixel` (for LEDs)
- This firmware file: `Wheelchair-Sim.ino`

---

## Electrode Placement

![Electrode Placement](electrode-placement.png)

**Typical placement:**
- **Positive** Near the outer corner of the left eye
- **Negative** Near the outer corner of the right eye
- **Reference:** Bony part behind the earlobe

---

## Controls & Key Mapping

| Action                | Key Sent |
|-----------------------|----------|
| Look Left (EOG)       | `a`      |
| Look Right (EOG)      | `d`      |
| Jaw Clench (single)   | `w`      |
| Jaw Clench (double)   | `s`      |

- **All keys are configurable** at the top of the code (`#define EOG_LEFT_KEY`, etc.)
---

## Debug Mode
- Set `#define DEBUG_LEVEL` at the top of the code:
  - `0` = Off (default)
  - `1` = Jaw debug (prints jaw envelope)
  - `2` = Eye debug (prints eye deviation)
- Debug output appears on Serial Monitor (115200 baud rate)

---

## Adjustable Parameters
You can tune these in the code to fit your needs:

| Name                    | Purpose                        | Default |
|-------------------------|--------------------------------|---------|
| `EYE_MOVEMENT_THRESHOLD`| Eye movement detection         | 150.0   |
| `JAW_THRESHOLD`         | Jaw clench detection           | 160.0   |
| `JAW_RELEASE_THRESHOLD` | Jaw release detection          | 70.0    |
| `MOVEMENT_DEBOUNCE_MS`  | Eye movement debounce (ms)     | 800     |
| `JAW_DEBOUNCE_MS`       | Jaw clench debounce (ms)       | 270     |
| `JAW_DOUBLE_WINDOW_MS`  | Double clench window (ms)      | 500     |
| `EYE_KEY_HOLD_MS`       | Eye keypress duration (ms)     | 200     |
| `JAW_HOLD_REPEAT_MS`    | Jaw key repeat interval (ms)   | 200     |

- **To change:** Edit the value in the code and re-upload.

---

## How It Works (Signal Pipeline)
1. **Raw ADC** (A0) sampled at 500 Hz
2. **Notch Filter** removes powerline noise (48–52 Hz)
3. **Eye Path:**
   - High-pass 1 Hz → Low-pass 10 Hz
   - Baseline tracker (rolling mean)
   - Detects left/right movement (compares to baseline)
4. **Jaw Path:**
   - High-pass 70 Hz
   - Envelope detector (rolling abs mean)
   - Detects single/double clench (with debounce)
5. **BLE Keyboard:** Sends keypresses to paired device
6. **NeoPixel LED:** Shows BLE connection status

---

## Getting Started
1. Connect electrodes and BioAmp snap cables as shown in the image
2. Flash the firmware to your NPG Lite board using a USB-C cable
3. Pair with your computer/phone via Bluetooth (shows as "NPG Lite GAMING")
4. Open a text editor or game and try the controls!

---

## More Info
- [Upside Down Labs](https://upsidedownlabs.tech)
- [NPG Lite Documentation](https://docs.upsidedownlabs.tech/hardware/bioamp/neuro-play-ground-lite/index.html)
- [Contact Support](mailto:contact@upsidedownlabs.tech)

---

