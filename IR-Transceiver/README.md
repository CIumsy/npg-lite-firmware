# IR Transceiver

Record and replay infrared remote signals on an NPG Lite (ESP32-C6), managed from a
browser over Bluetooth.

Hold the user button to record a signal from any IR remote, name it in the web app,
then replay it with a short press of the button or the Fire button in the app.

## Hardware

| Part | Link |
| --- | --- |
| Adafruit IR Transceiver | [adafruit.com/product/5990](https://www.adafruit.com/product/5990) |
| 4-pin JST PH to JST SH cable, STEMMA to QT / Qwiic, 200mm | [adafruit.com/product/4424](https://www.adafruit.com/product/4424) |
| NPG Lite kit | [upsidedownlabs.in](https://www.upsidedownlabs.in/shop?search=neuro+playground) |

### Wiring

| Adafruit IR Transceiver | Pin | Pin on NPG Lite |
| --- | --- | --- |
| Out | SDA | 23 |
| In | SCL | 22 |
| Gnd | Gnd | Gnd |
| Vin | 3v3 | 3v3 |

SDA and SCL are just the wire names on the STEMMA cable. They carry the transceiver's
Out and In signals, and the firmware drives them as ordinary GPIO rather than as an
I2C bus. Both pins are macros at the top of `IR-Transceiver.ino`.

### Status LEDs

The onboard NeoPixel ring reports two things at a glance. Pixel numbering
matches the rest of the NPG Lite firmware.

| Pixel | Reports | Colours |
| --- | --- | --- |
| 0 | Bluetooth | red not connected, green connected, blue flash when a command fires |
| 5 | Battery | red at or below 20%, amber at or below 70%, green above |

Battery is sampled from `A6` in millivolts every 100 ms, averaged and mapped
through a voltage lookup table every 30 seconds, the same one the other NPG Lite
sketches use. The percentage only rises after three consecutive higher readings,
so the colour does not flicker under load.

Nothing is written to the ring while the receiver is armed. Pushing pixel data
briefly disables interrupts, which can cost the IR receiver an edge mid-capture,
so the ring is left alone until recording ends.

### Checking the transceiver is connected

The firmware cannot tell you this. The module is wired as two plain GPIO lines
with nothing to interrogate, so read its own indicators instead.

| Indicator | Meaning |
| --- | --- |
| Green power LED | The module has 3V3 and ground. If this is dark, check the cable. |
| OUT and IN LEDs | Each flashes once as a signal passes through it. |

Fire a saved command with a short press of the user button. Both the OUT and IN
LEDs should blink once. Green power LED but no blink means the module is powered
yet its signal pins are not reaching the board, so check pins 22 and 23.

### The user button

The sketch reads GPIO 9, which is the boot button on the NPG Lite and on most
ESP32-C6 boards. On a different dev board this pin will be wrong. Change
`USER_BTN_PIN` at the top of the sketch to whichever button that board exposes.
Many other ESP32 boards put the boot button on GPIO 0.

## Flashing

Two ways. The web flasher is quicker if you only want to run the firmware. Use the
Arduino IDE if you plan to change it.

### Web flasher

Use the [NPG Lite Flasher Web](https://upsidedownlabs.github.io/NPG-Lite-Flasher-Web/).

1. Connect the NPG Lite, or another ESP32-C6 board, over USB.
2. Press **Connect** and pick the **USB JTAG** serial device.
3. Press **Get from GitHub** and choose the **IR-Transceiver** firmware.
4. Flash it.

### Arduino IDE

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install these from the Library Manager, under **Tools -> Manage Libraries**.
   - `IRremoteESP8266` (tested against 2.9.0)
   - `Adafruit NeoPixel`
3. Install **ESP32 (version 3.2.0)** by Espressif Systems from the Boards Manager.
   Go to **Tools -> Board -> Boards Manager**.
4. Open `IR-Transceiver.ino` from this folder. If you would rather paste the code into
   a new sketch, save that sketch as `IR-Transceiver` so the folder and the file share
   a name, which the Arduino IDE requires.
5. Open the board selector dropdown at the top of the window, the one that reads
   **Select Board**, and pick your board's COM port. It may show as an ESP32 Family
   Device.
6. Go to **Tools -> Board -> ESP32 -> ESP32C6 Dev Module**.
7. Go to **Tools -> Partition Scheme -> Huge APP (3MB No OTA/1MB SPIFFS)**.
8. Hit the upload button.

Step 7 is not optional. On the default partition scheme the sketch fills 95% of
program storage and will overflow as soon as anything is added. Huge APP brings it to
39% and still leaves 1 MB for signal storage.

## Using the web app

Open
[NPG-Lite-Arduino-Firmware in BioAmp Arduino Firmware Explorer](https://upsidedownlabs.github.io/BioAmp-Arduino-Firmware-Explorer/?owner=upsidedownlabs&repo=npg-lite-firmware)
and go to **web app** under **IR-Transceiver**.

Alternatively, download this repo and open `index.html` from the `IR-Transceiver`
folder in your browser.

Either way you need a Chromium based browser like Chrome, Brave or Edge, or any
browser with Web Bluetooth. Firefox and Safari do not support it.

Press Connect and pick **NPG-IR**. The command list loads automatically.

To record a signal, hold the user button on the board for 1.5 seconds and let go.
Recording latches on, so you do not have to keep holding, and the app shows a
listening overlay with a countdown. Point a remote at the board and press a key.
The capture dialog opens in the app.

Recording ends when a signal arrives, after 15 seconds, or when you cancel it with a
short press of the button or the Cancel button in the app. A capture lives in RAM only
until you name it, so it is lost on reboot if you never save it.

The info button in the header explains the same thing inside the app, along with the
storage limits and the Erase all action.

## Layout

```
IR-Transceiver.ino   firmware
index.html           markup
css/style.css        design tokens and all styling
js/ble.js            Bluetooth transport and wire protocol
js/app.js            UI and state
assets/              UDL logo, black for light theme and white for dark
```

`ble.js` never touches the DOM and `app.js` never touches Bluetooth. To change the
protocol, only `ble.js` and the firmware need to agree.

## Storage

Each command is split across two stores on the board.

| Store | Holds | Capacity |
| --- | --- | --- |
| NVS | names, active slot | 20 KB, roughly 250 names |
| LittleFS | captured waveforms | 1 MB, roughly 250 signals |

A slot counts as used purely because a name key exists in NVS, so NVS is the index and
LittleFS is the payload. The build caps this at 50 commands of 16 characters via
`MAX_COMMANDS` and `MAX_NAME_LEN`. Raising `MAX_COMMANDS` is safe up to 255, which is
where the single-byte slot id in the protocol runs out.

Names are capped at 16 characters so every notification fits the 20 byte payload that
the default Bluetooth MTU guarantees, with no dependency on MTU negotiation.

Both stores survive a firmware upload. Use **Erase all** in the app to clear them.

## Protocol

Two characteristics on service `12345678-1234-1234-1234-1234567890ab`, one for
notifications and one for writes.

| App to board | Payload |
| --- | --- |
| `0x01` set active | id |
| `0x02` delete | id |
| `0x03` rename | id, name |
| `0x04` save capture as new | name |
| `0x05` save capture over slot | id, name |
| `0x06` request list | |
| `0x07` fire | id |
| `0x08` erase everything | |
| `0x09` cancel recording | |
| `0x0A` discard pending capture | |

| Board to app | Payload |
| --- | --- |
| `0x12` list entry | id, active, nameLen, name |
| `0x14` end of list | |
| `0x20` capture ready | isDup, dupId, protocol, bits, value |
| `0x21` saved | id, nameLen, name |
| `0x22` deleted | id |
| `0x23` active changed | id |
| `0x24` ok | |
| `0x25` failed | |
| `0x26` erased | |
| `0x27` recording started | timeout in seconds |
| `0x28` recording ended | reason: 0 timeout, 1 button, 2 app |

Firing, listing, erasing and cancelling a recording are handled on the main loop
rather than inside the Bluetooth callback, because that callback runs on a task with
a much smaller stack than the IR and filesystem code needs.

---

Making neuroscience affordable and accessible for everyone

[Upside Down Labs](https://upsidedownlabs.in)
