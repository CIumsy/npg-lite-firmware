/* Bluetooth transport and wire protocol.
   Knows nothing about the DOM. Emits decoded events to subscribers. */

const BLE = (() => {

  const SVC_UUID    = '12345678-1234-1234-1234-1234567890ab';
  const NOTIFY_UUID = '12345678-1234-1234-1234-1234567890ac';
  const WRITE_UUID  = '12345678-1234-1234-1234-1234567890ad';

  const DEVICE_NAME = 'NPG-IR';
  const MAX_NAME    = 16;

  // board -> app
  const EV = {
    LIST_ENTRY: 0x12,
    LIST_END:   0x14,
    CAPTURE:    0x20,
    SAVED:      0x21,
    DELETED:    0x22,
    ACTIVE:     0x23,
    OK:         0x24,
    FAIL:       0x25,
    WIPED:      0x26,
    LISTENING:  0x27,
    LISTEN_END: 0x28,
  };

  // app -> board
  const CMD = {
    SET_ACTIVE: 0x01,
    DELETE:     0x02,
    RENAME:     0x03,
    SAVE_NEW:   0x04,
    SAVE_OVER:  0x05,
    GET_LIST:   0x06,
    FIRE:       0x07,
    WIPE:       0x08,
    CANCEL_REC: 0x09,
  };

  const PROTOCOLS = {
    0: 'Unknown', 1: 'RC5', 2: 'RC6', 3: 'NEC', 4: 'Sony', 5: 'Panasonic',
    6: 'JVC', 7: 'Samsung', 8: 'Whynter', 9: 'Sanyo', 10: 'LG',
    12: 'Mitsubishi', 15: 'Coolix', 16: 'Daikin', 18: 'Kelvinator',
    20: 'Mitsubishi AC', 24: 'Gree', 29: 'Fujitsu AC', 32: 'Hitachi AC',
    45: 'Toshiba AC', 48: 'Trotec',
  };

  const decoder = new TextDecoder();
  const encoder = new TextEncoder();

  let device = null;
  let writeChar = null;
  const handlers = {};

  function on(event, fn) {
    (handlers[event] ||= []).push(fn);
  }

  function emit(event, payload) {
    (handlers[event] || []).forEach(fn => fn(payload));
  }

  function protocolName(code) {
    return PROTOCOLS[code] || `Protocol ${code}`;
  }

  /* Names are capped at 16 bytes so every packet fits the 20 byte
     payload guaranteed by the default BLE MTU. Slice by encoded
     bytes rather than characters so multi-byte input cannot overrun. */
  function encodeName(text) {
    const bytes = encoder.encode(text);
    return Array.from(bytes.slice(0, MAX_NAME));
  }

  function isSupported() {
    return typeof navigator !== 'undefined' && !!navigator.bluetooth;
  }

  function isConnected() {
    return !!(device && device.gatt.connected && writeChar);
  }

  async function connect() {
    if (!isSupported()) {
      throw new Error('Web Bluetooth is not available in this browser');
    }

    device = await navigator.bluetooth.requestDevice({
      filters: [{ name: DEVICE_NAME }],
      optionalServices: [SVC_UUID],
    });

    device.addEventListener('gattserverdisconnected', () => {
      writeChar = null;
      emit('disconnect');
    });

    const server  = await device.gatt.connect();
    const service = await server.getPrimaryService(SVC_UUID);

    const notifyChar = await service.getCharacteristic(NOTIFY_UUID);
    await notifyChar.startNotifications();
    notifyChar.addEventListener('characteristicvaluechanged', onNotify);

    writeChar = await service.getCharacteristic(WRITE_UUID);
    emit('connect', device.name || DEVICE_NAME);
  }

  function disconnect() {
    if (device && device.gatt.connected) device.gatt.disconnect();
  }

  async function send(bytes) {
    if (!writeChar) throw new Error('Not connected');
    await writeChar.writeValueWithResponse(new Uint8Array(bytes));
  }

  function onNotify(event) {
    const d = new Uint8Array(event.target.value.buffer);
    if (d.length === 0) return;

    switch (d[0]) {

      case EV.LIST_ENTRY: {
        const nameLen = d[3];
        emit('listEntry', {
          id: d[1],
          active: d[2] === 1,
          name: decoder.decode(d.slice(4, 4 + nameLen)),
        });
        break;
      }

      case EV.LIST_END:
        emit('listEnd');
        break;

      case EV.CAPTURE: {
        let value = 0n;
        for (let i = 0; i < 8; i++) value |= BigInt(d[7 + i]) << BigInt(8 * i);
        emit('capture', {
          isDuplicate: d[1] === 1,
          duplicateId: d[2] === 0xFF ? -1 : d[2],
          protocol: d[3] | (d[4] << 8),
          bits: d[5] | (d[6] << 8),
          value: value.toString(16).toUpperCase(),
        });
        break;
      }

      case EV.SAVED: {
        const nameLen = d[2];
        emit('saved', {
          id: d[1],
          name: decoder.decode(d.slice(3, 3 + nameLen)),
        });
        break;
      }

      case EV.DELETED:    emit('deleted', d[1]);         break;
      case EV.ACTIVE:     emit('active',  d[1]);         break;
      case EV.OK:         emit('fired');                 break;
      case EV.FAIL:       emit('failed');                break;
      case EV.WIPED:      emit('wiped');                 break;
      case EV.LISTENING:  emit('listening', d[1] || 15); break;
      case EV.LISTEN_END: emit('listenEnd');             break;
    }
  }

  /* commands ------------------------------------------------- */

  const setActive = id           => send([CMD.SET_ACTIVE, id]);
  const remove    = id           => send([CMD.DELETE, id]);
  const rename    = (id, name)   => send([CMD.RENAME, id, ...encodeName(name)]);
  const saveNew   = name         => send([CMD.SAVE_NEW, ...encodeName(name)]);
  const saveOver  = (id, name)   => send([CMD.SAVE_OVER, id, ...encodeName(name)]);
  const getList   = ()           => send([CMD.GET_LIST]);
  const fire      = id           => send([CMD.FIRE, id]);
  const wipe      = ()           => send([CMD.WIPE]);
  const cancelRec = ()           => send([CMD.CANCEL_REC]);

  return {
    MAX_NAME, DEVICE_NAME,
    on, connect, disconnect, isConnected, isSupported, protocolName,
    setActive, remove, rename, saveNew, saveOver, getList, fire, wipe, cancelRec,
  };

})();
