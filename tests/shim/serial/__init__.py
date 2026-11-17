"""The part of pyserial that ArduPilot's uploader.py uses, on termios.  Tests only: it lets
uploader.py run against tests/sim_pty without installing pyserial (POSIX)."""

import fcntl
import os
import select
import struct
import termios
import time


class SerialException(OSError):
    pass


class SerialTimeoutException(SerialException):
    pass


class Serial:
    def __init__(self, port=None, baudrate=115200, timeout=None, write_timeout=None, exclusive=None, **kw):
        self.port = port
        self._baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.fd = None
        if port is not None:
            self.open()

    def open(self):
        try:
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise SerialException(str(e))
        a = termios.tcgetattr(self.fd)
        a[0] = 0                                                 # iflag
        a[1] = 0                                                 # oflag
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL       # cflag
        a[3] = 0                                                 # lflag: raw, no echo
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    @property
    def is_open(self):
        return self.fd is not None

    def isOpen(self):
        return self.is_open

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self, value):
        self._baudrate = value               # a pseudo-terminal has no line rate

    def read(self, size=1):
        out = b''
        deadline = None if self.timeout is None else time.monotonic() + self.timeout
        while len(out) < size:
            # timeout 0 is non-blocking: still take whatever is already there
            left = None if deadline is None else max(0.0, deadline - time.monotonic())
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                break
            try:
                chunk = os.read(self.fd, size - len(out))
            except BlockingIOError:
                continue
            if not chunk:
                break
            out += chunk
        return out

    def write(self, data):
        data = bytes(data)
        done = 0
        while done < len(data):
            select.select([], [self.fd], [], self.write_timeout)
            try:
                done += os.write(self.fd, data[done:])
            except BlockingIOError:
                pass
        return done

    def flush(self):
        pass

    def flushInput(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    reset_input_buffer = flushInput

    @property
    def in_waiting(self):
        return struct.unpack('I', fcntl.ioctl(self.fd, termios.FIONREAD, b'\0\0\0\0'))[0]

    def inWaiting(self):
        return self.in_waiting

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
