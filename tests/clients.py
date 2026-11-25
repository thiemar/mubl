"""Host-side protocol clients that talk to the simulation directly (no ports, simulated time)."""

import struct

INSYNC, EOC, OK, FAILED, INVALID = 0x12, 0x20, 0x10, 0x11, 0x13
GET_SYNC, GET_DEVICE, CHIP_ERASE, PROG_MULTI, GET_CRC = 0x21, 0x22, 0x23, 0x27, 0x29
GET_SN, GET_CHIP, BOOT, SET_BAUD = 0x2B, 0x2C, 0x30, 0x33


class NoReply(Exception):
    pass


class SerialClient:
    """The PX4/ArduPilot bootloader protocol, as uploader.py speaks it."""

    def __init__(self, sim, port=0, baud=115200):
        self.sim, self.port, self.baud = sim, port, baud

    def send(self, data):
        self.sim.uart_send(self.port, bytes(data), self.baud)

    def recv(self):
        return self.sim.uart_recv(self.port, self.baud)

    def command(self, data, reply=0, timeout_ms=3000):
        self.send(data)
        out = b''
        for _ in range(timeout_ms):
            if self.sim.run(1) != 1:
                break
            out += self.recv()
            if len(out) >= reply + 2:
                break
        out += self.recv()
        if len(out) < reply + 2:
            raise NoReply('%r -> %r' % (bytes(data[:4]), out))
        if out[reply] != INSYNC:
            raise NoReply('bad sync in %r' % out)
        return out[:reply], out[reply + 1]

    def sync(self):
        self.recv()                                     # like uploader.py: drop what came before
        return self.command([GET_SYNC, EOC])[1]

    def device(self, arg):
        data, status = self.command([GET_DEVICE, arg, EOC], 4)
        return struct.unpack('<I', data)[0] if status == OK else None

    def identify(self):
        self.sync()
        return {'bl_rev': self.device(1), 'board_id': self.device(2), 'board_rev': self.device(3),
                'fw_size': self.device(4)}

    def erase(self):
        return self.command([CHIP_ERASE, EOC], timeout_ms=30000)[1]

    def program(self, chunk):
        return self.command(bytes([PROG_MULTI, len(chunk)]) + bytes(chunk) + bytes([EOC]))[1]

    def crc(self):
        data, status = self.command([GET_CRC, EOC], 4, timeout_ms=10000)
        return struct.unpack('<I', data)[0] if status == OK else None

    def boot(self):
        return self.command([BOOT, EOC])[1]

    def upload(self, image, boot=True):
        info = self.identify()
        assert self.erase() == OK
        for off in range(0, len(image), 252):
            chunk = image[off:off + 252]
            chunk += b'\xff' * (-len(chunk) % 4)
            assert self.program(chunk) == OK, off
        return info if not boot else (info, self.boot())


class UsbSerialClient(SerialClient):
    """The same protocol over the simulated USB host's CDC-ACM port."""

    def __init__(self, sim):
        super().__init__(sim)

    def send(self, data):
        self.sim.usb_send(bytes(data))

    def recv(self):
        return self.sim.usb_recv()


class SimSrxl2Transport:
    """mubl_srxl2.Updater transport on the simulation's UART port."""

    def __init__(self, sim, port=0):
        self.sim, self.port, self.baud = sim, port, 115200

    def now(self):
        return self.sim.now() / 1000.0

    def write(self, data):
        self.sim.uart_send(self.port, bytes(data), self.baud)

    def read(self, timeout):
        out = b''
        for _ in range(max(1, int(timeout * 1000))):
            if self.sim.run(1) == 0:                  # powered off: time still passes for the host
                self.sim.set_time(self.sim.now() + 1)
                continue
            out += self.sim.uart_recv(self.port, self.baud)
            if out:
                break
        return out

    def set_baud(self, baud):
        self.baud = baud
