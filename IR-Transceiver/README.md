# IR BLE Controller

Record and replay infrared remote signals on an NPG Lite (ESP32-C6), managed from a
browser over Bluetooth.

Hold the user button to capture a signal from any IR remote, name it in the web app,
then replay it with a short press of the button or the Fire button in the app.

## Hardware

| Signal | Pin |
| --- | --- |
| IR receiver out | GPIO 23 |
| IR emitter in | GPIO 22 |
| User button | GPIO 9 |

All three are macros at the top of `IR_BLE_Controller.ino`.

## Building the firmware

Board: **ESP32C6 Dev Module**.

Partition scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**. This matters. The default
scheme leaves the sketch at 95% of program storage, which will overflow as soon as
anything is added. Huge APP brings it to 39% and still leaves 1 MB for signal storage.

Required library: `IRremoteESP8266` (tested against 2.9.0).

## Using the web app

Open `web/index.html` in any Chromium based browser like Chrome, brave or Edge (or any web-ble supported browser). Firefox and Safari have no Web Bluetooth support.

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
IR_BLE_Controller.ino   firmware
web/index.html          markup
web/css/style.css       design tokens and all styling
web/js/ble.js           Bluetooth transport and wire protocol
web/js/app.js           UI and state
web/assets/             UDL logo, black for light theme and white for dark
_old/                   previous single-file web app, kept for reference
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

Both stores survive a firmware upload. Erase all in the app clears them, or:

```
esptool.py --chip esp32c6 -p COM5 erase_flash
```

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
