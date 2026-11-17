#!/usr/bin/env python3
"""Update a µBL device over SRXL2, acting as the bus master.

    mubl_srxl2.py /dev/ttyUSB0 app.bin [--device 0x40] [--fast] [--no-echo]

Connect the adapter's TX and RX together to the SRXL2 signal wire (a resistor of about 1 kΩ in
the TX line is the usual half-duplex arrangement); the bytes the adapter sends then come back on
its RX and are discarded (--no-echo if your adapter does not echo).

Sequence: Handshake with the device (SRXL2 Rev K 7.2), optionally switch the bus to 400000 baud,
then the µBL update packet (type 0xB0): INFO, BEGIN, WRITE (64-byte chunks), FINISH with the
image's zlib CRC-32, BOOT.  A request without an answer is repeated with the same sequence
number; the device answers a repeat without executing it twice.
"""

import argparse
import os
import struct
import sys
import time
import zlib

SRXL_ID = 0xA6
TYPE_HANDSHAKE = 0x21
TYPE_DFU = 0xB0
OP_INFO, OP_BEGIN, OP_WRITE, OP_FINISH, OP_BOOT = 1, 2, 3, 4, 5
CHUNK = 64
STATUS = {0: 'ok', 1: 'busy (another link is updating)', 2: 'out of sequence', 3: 'image too large',
          4: 'flash error', 5: 'image rejected', 6: 'no update in progress', 7: 'bad request'}
APP_STATUS = {0: 'valid', 1: 'empty', 2: 'bad vectors', 3: 'no descriptor', 4: 'bad size', 5: 'bad CRC',
              6: 'wrong board', 7: 'flash error'}


class Error(Exception):
    pass


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def packet(ptype, payload):
    p = bytes([SRXL_ID, ptype, len(payload) + 5]) + bytes(payload)
    return p + struct.pack('>H', crc16(p))


class Parser:
    """Splits a byte stream into CRC-checked SRXL2 packets."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([SRXL_ID]))
            if i < 0:
                self.buf.clear()
                return out
            del self.buf[:i]
            if len(self.buf) < 3:
                return out
            n = self.buf[2]
            if n < 5 or n > 80:
                del self.buf[:1]
                continue
            if len(self.buf) < n:
                return out
            p = bytes(self.buf[:n])
            if crc16(p) == 0:
                out.append(p)
                del self.buf[:n]
            else:
                del self.buf[:1]


class SerialTransport:
    def __init__(self, port, echo=True):
        import serial
        self.port = serial.Serial(port, 115200, timeout=0)
        self.echo = echo

    def write(self, data):
        self.port.write(data)
        self.port.flush()
        if self.echo:                    # single wire: our own bytes come back first
            got = b''
            deadline = time.time() + 0.1 + len(data) * 12 / self.port.baudrate
            while len(got) < len(data) and time.time() < deadline:
                got += self.port.read(len(data) - len(got))

    def now(self):
        return time.monotonic()

    def read(self, timeout):
        deadline = time.time() + timeout
        data = b''
        while time.time() < deadline:
            data += self.port.read(256)
            if data:
                time.sleep(0.002)
                data += self.port.read(256)
                return data
            time.sleep(0.001)
        return data

    def set_baud(self, baud):
        self.port.baudrate = baud


class Updater:
    def __init__(self, transport, device=0x40, master=0x21, log=print):
        self.t = transport
        self.device = device
        self.master = master
        self.seq = int.from_bytes(os.urandom(1), 'little')
        self.parser = Parser()
        self.log = log
        self.fast = False

    def _wait(self, match, timeout):
        deadline = self.t.now() + timeout
        while self.t.now() < deadline:
            for p in self.parser.feed(self.t.read(min(deadline - self.t.now(), 0.05))):
                r = match(p)
                if r is not None:
                    return r
        return None

    def handshake(self, retries=10):
        hs = packet(TYPE_HANDSHAKE, bytes([self.master, self.device, 10, 1, 0]) + bytes(4))
        for _ in range(retries):
            self.t.write(hs)
            r = self._wait(lambda p: p if p[1] == TYPE_HANDSHAKE and p[3] == self.device and p[4] == self.master
                           else None, 0.1)
            if r is not None:
                return {'priority': r[5], 'fast': bool(r[6] & 1), 'info': r[7],
                        'uid': struct.unpack_from('<I', r, 8)[0]}
        raise Error('no handshake from device 0x%02X' % self.device)

    def set_fast(self):
        """Broadcast the 400000 baud handshake (7.2.1 step 6) and switch."""
        self.t.write(packet(TYPE_HANDSHAKE, bytes([self.master, 0xFF, 10, 1, 0]) + bytes(4)))
        self.t.read(0.01)                     # give the device time to switch before we do
        self.t.set_baud(400000)
        self.fast = True

    def _renegotiate(self):
        """A device that heard nothing for a while has gone back to 115200: handshake again."""
        self.t.set_baud(115200)
        self.handshake()
        self.set_fast()

    def request(self, op, data=b'', timeout=1.0, retries=5):
        seq = self.seq = (self.seq + 1) & 0xFF
        req = packet(TYPE_DFU, bytes([self.device, self.master, op, seq]) + bytes(data))

        def match(p):
            if p[1] == TYPE_DFU and len(p) >= 10 and p[3] == self.master and p[4] == self.device and \
                    p[5] == (op | 0x80) and p[6] == seq:
                return p[7], p[8:-2]
            return None
        for _ in range(retries):
            self.t.write(req)
            r = self._wait(match, timeout)
            if r is not None:
                return r
            if self.fast:
                self._renegotiate()
        raise Error('no answer to request %d' % op)

    def info(self):
        status, d = self.request(OP_INFO)
        if status != 0 or len(d) < 35:
            raise Error('INFO failed')
        f = struct.unpack_from('<BBBIIBBII', d, 0)
        return {'protocol': f[0], 'app_status': f[1], 'app_format': f[2], 'board_id': f[3], 'region': f[4],
                'version': (f[5], f[6]), 'vcs_commit': f[7], 'app_size': f[8], 'uid': d[21:33].hex(),
                'bootloader': (d[33], d[34])}

    def _check(self, status, what):
        if status != 0:
            raise Error('%s: %s' % (what, STATUS.get(status, status)))

    def update(self, image, board_id=None, progress=None):
        info = self.info()
        if board_id is not None and info['board_id'] != board_id:
            raise Error('device board ID %d is not %d' % (info['board_id'], board_id))
        if len(image) > info['region']:
            raise Error('image of %d bytes does not fit in %d' % (len(image), info['region']))
        status, _ = self.request(OP_BEGIN, struct.pack('<I', len(image)), timeout=5.0)
        self._check(status, 'BEGIN')
        for off in range(0, len(image), CHUNK):
            status, d = self.request(OP_WRITE, struct.pack('<I', off) + image[off:off + CHUNK], timeout=3.0)
            self._check(status, 'WRITE at %d' % off)
            if progress:
                progress(off + CHUNK, len(image))
        crc = zlib.crc32(bytes(image)) & 0xFFFFFFFF
        status, d = self.request(OP_FINISH, struct.pack('<II', len(image), crc), timeout=10.0)
        if status != 0:
            raise Error('FINISH: %s (application %s)' % (STATUS.get(status, status),
                                                          APP_STATUS.get(d[0] if d else -1, '?')))
        return info

    def boot(self):
        status, _ = self.request(OP_BOOT)
        self._check(status, 'BOOT')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port')
    ap.add_argument('image')
    ap.add_argument('--device', type=lambda v: int(v, 0), default=0x40, help='SRXL2 device ID (default 0x40)')
    ap.add_argument('--master', type=lambda v: int(v, 0), default=0x21, help='our device ID as bus master')
    ap.add_argument('--board-id', type=int, help='refuse a device with another board ID')
    ap.add_argument('--fast', action='store_true', help='switch the bus to 400000 baud')
    ap.add_argument('--no-echo', action='store_true', help='the adapter does not echo what it sends')
    ap.add_argument('--no-boot', action='store_true')
    args = ap.parse_args()
    image = open(args.image, 'rb').read()
    u = Updater(SerialTransport(args.port, echo=not args.no_echo), args.device, args.master)
    hs = u.handshake()
    print('device 0x%02X answered (uid %08x)' % (args.device, hs['uid']))
    if args.fast and hs['fast']:
        u.set_fast()
        u.handshake()

    def progress(done, total):
        sys.stdout.write('\r%d / %d bytes' % (min(done, total), total))
        sys.stdout.flush()
    info = u.update(image, args.board_id, progress)
    print('\nwritten; board %d, region %d bytes' % (info['board_id'], info['region']))
    if not args.no_boot:
        u.boot()
        print('booting')


if __name__ == '__main__':
    try:
        main()
    except Error as e:
        sys.exit('error: %s' % e)
