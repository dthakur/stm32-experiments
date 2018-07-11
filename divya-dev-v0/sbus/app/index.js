const assert = require('assert');
const SerialPort = require('serialport');

const SBUS_FRAME_BEGIN_BYTE = 0x0f;
const SBUS_FRAME_END_BYTE = 0x00;
const SBUS_LOST_FRAME_MASK = 0x04;
const SBUS_FAIL_SAFE_MASK = 0x08;

function onSbus(bytes) {
  if (bytes[0] !== SBUS_FRAME_BEGIN_BYTE) {
    return;
  }

  if (bytes[bytes.length - 1] !== SBUS_FRAME_END_BYTE) {
    return;
  }

  assert(bytes.length === 25, bytes.length);

  return {
    chan0: ((bytes[1]         | bytes[2]  << 8)                    & 0x07ff),
    chan1: ((bytes[2]   >>> 3 | bytes[3]  << 5)                    & 0x07ff),
    chan2: ((bytes[3]   >>> 6 | bytes[4]  << 2  | bytes[5]  << 10) & 0x07ff),
    chan3: ((bytes[5]   >>> 1 | bytes[6]  << 7)                    & 0x07ff),
    chan4: ((bytes[6]   >>> 4 | bytes[7]  << 4)                    & 0x07ff),
    chan5: ((bytes[7]   >>> 7 | bytes[8]  << 1  | bytes[9]  << 9)  & 0x07ff),
    chan6: ((bytes[9]   >>> 2 | bytes[10] << 6)                    & 0x07ff),
    chan7: ((bytes[10]  >>> 5 | bytes[11] << 3)                    & 0x07ff),
    chan8: ((bytes[12]        | bytes[13] << 8)                    & 0x07ff),
    chan9: ((bytes[13]  >>> 3 | bytes[14] << 5)                    & 0x07ff),
    chan10: ((bytes[14] >>> 6 | bytes[15] << 2  | bytes[16] << 10) & 0x07ff),
    chan11: ((bytes[16] >>> 1 | bytes[17] << 7)                    & 0x07ff),
    chan12: ((bytes[17] >>> 4 | bytes[18] << 4)                    & 0x07ff),
    chan13: ((bytes[18] >>> 7 | bytes[19] << 1  | bytes[20] <<  9) & 0x07ff),
    chan14: ((bytes[20] >>> 2 | bytes[21] << 6)                    & 0x07ff),
    chan15: ((bytes[21] >>> 5 | bytes[22] << 3)                    & 0x07ff),
    flags: {
      lostFrame: (bytes[23] & SBUS_LOST_FRAME_MASK) > 0,
      failsafe: (bytes[23] & SBUS_FAIL_SAFE_MASK) > 0
    }
  };
}

function main() {
  const port = new SerialPort('/dev/tty.usbmodem14111', {
    baudRate: 115200
  });
  
  port.on('data', function (data) {
    response = onSbus(data);
    console.log(JSON.stringify(response));
  });  
}

main()