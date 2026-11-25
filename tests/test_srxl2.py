"""SRXL2: the handshake against Spektrum's reference slave, and the µBL update packet.

The public reference implementation (tests/third_party/srxl2, MIT) contains only the slave side,
so the handshake is checked differentially: the reference slave and the bootloader, configured
alike, receive the same bus master traffic in lockstep and must answer it the same way.
"""

import ctypes
import os
import struct
import subprocess
import sys
import unittest
import zlib

from clients import OK, SerialClient, SimSrxl2Transport
from sim import (APP_EMPTY, APP_OK, BUILD, CC, ROOT, STATE_APP, STATE_BOOTLOADER, STAY_HOST, Sim, make_image)

sys.path.insert(0, os.path.join(ROOT, 'tools'))
import mubl_srxl2 as srxl2  # noqa: E402

DEVICE = 0x40
SRXL2_ONLY = {'MUBL_SERIAL_PORT': 1}          # serial elsewhere: port 0 is SRXL2 alone


def reference():
    out = os.path.join(BUILD, 'libsrxl2ref.so')
    srcs = [os.path.join(ROOT, 'tests', 'third_party', 'srxl2', 'spm_srxl.c'), os.path.join(ROOT, 'tests', 'srxl2_ref.c')]
    if not os.path.exists(out) or os.path.getmtime(out) < max(os.path.getmtime(s) for s in srcs):
        os.makedirs(BUILD, exist_ok=True)
        subprocess.check_call([CC, '-std=c11', '-O1', '-w', '-fPIC', '-shared',
                               '-I' + os.path.join(ROOT, 'tests', 'third_party', 'srxl2')] + srcs + ['-o', out])
    lib = ctypes.CDLL(out)
    lib.ref_init.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint8]
    lib.ref_feed.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.ref_run.argtypes = [ctypes.c_int]
    lib.ref_take.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]
    lib.ref_baud.restype = ctypes.c_uint32
    return lib


def handshake(src, dest, baud=1, priority=10):
    return srxl2.packet(0x21, bytes([src, dest, priority, baud, 0]) + bytes(4))


class Lockstep:
    """A scripted bus master, the bootloader and the reference slave on one wire."""

    def __init__(self, device=DEVICE):
        self.sim = Sim(1, SRXL2_ONLY).fresh()
        self.ref = reference()
        uid = struct.unpack('<III', bytes(range(0xA0, 0xAC)))
        self.ref.ref_init(device, 1, 0, uid[0] ^ uid[1] ^ uid[2], 1)
        self.sim.boot()
        self.t = 0
        self.t0 = self.sim.now()
        self.device_time = {}           # packet -> the device's clock when it was sent
        self.master_baud = 115200
        self.log = {'mubl': [], 'ref': []}
        self.parsers = {'mubl': srxl2.Parser(), 'ref': srxl2.Parser()}

    def send(self, data):
        if self.sim.uart_baud(0) == self.master_baud:
            self.sim.uart_send(0, data, self.master_baud)
        if self.ref.ref_baud() == self.master_baud:
            self.ref.ref_feed(data, len(data))

    def run(self, ms):
        buf = (ctypes.c_uint8 * 8192)()
        for _ in range(ms):
            self.t += 1
            self.sim.run(1)
            self.ref.ref_run(1)
            out = {'mubl': self.sim.uart_recv(0, 0)}
            n = self.ref.ref_take(buf, 8192)
            out['ref'] = bytes(buf[:n])
            for who, data in out.items():
                for p in self.parsers[who].feed(data):
                    self.log[who].append((self.t, p))
                    if who == 'mubl':
                        self.device_time.setdefault(len(self.log[who]) - 1, self.sim.now() - self.t0)

    def since(self, who, t):
        return [p for (when, p) in self.log[who] if when > t]


class Handshake(unittest.TestCase):
    def test_announce_matches_reference(self):
        b = Lockstep()
        b.run(300)
        first = {who: b.log[who][0] for who in b.log}
        self.assertEqual(first['mubl'][1], first['ref'][1])       # identical bytes
        self.assertEqual(first['mubl'][1][4], 0)                  # to device 0: an unprompted announce
        for who in b.log:
            self.assertGreaterEqual(first[who][0], 50, who)       # after listening for 50 ms
            self.assertLessEqual(first[who][0], 52, who)
        # the bootloader announces every 50 ms for 200 ms (spec 7.2.1 step 3) and then stops; on its
        # own clock (which runs ahead of these ticks when sending takes time: MUBL_SIM_BLOCKING)
        times = [b.device_time[i] for i in range(len(b.log['mubl']))]
        self.assertEqual(len(times), 4, times)
        for got, want in zip(times, (50, 100, 150, 200)):
            self.assertLessEqual(abs(got - want), 2, times)

    def test_reply_to_master_matches_reference(self):
        b = Lockstep()
        b.run(300)
        t = b.t
        # The reference counts its own announcement as the answer to a master's first poll and
        # answers the next (and repeats it on every srxlRun call, here each millisecond); the
        # bootloader answers every poll once.  Poll twice, as a master does.
        for _ in range(2):
            b.send(handshake(0x21, DEVICE))
            b.run(5)
        mubl = [p for p in b.since('mubl', t) if p[4] == 0x21]
        ref = [p for p in b.since('ref', t) if p[4] == 0x21]
        self.assertEqual(len(mubl), 2)
        self.assertTrue(ref)
        self.assertEqual(mubl[0], ref[0])                       # identical reply
        self.assertEqual(mubl[0][3], DEVICE)

    def test_handshake_to_other_device_ignored(self):
        b = Lockstep()
        b.run(300)
        t = b.t
        b.send(handshake(0x21, 0x60))
        b.run(5)
        self.assertEqual(b.since('mubl', t), [])
        self.assertEqual([p for p in b.since('ref', t) if p[4] == 0x21], [])

    def test_broadcast_sets_baud_and_silence_restores(self):
        b = Lockstep()
        b.run(300)
        b.send(handshake(0x21, DEVICE))
        b.run(2)
        b.send(handshake(0x21, 0xFF, baud=1))
        b.run(2)
        self.assertEqual(b.sim.uart_baud(0), 400000)
        self.assertEqual(b.ref.ref_baud(), 400000)
        b.run(60)                                   # the master stops: back to 115200 (7.2.1 step 6)
        self.assertEqual(b.sim.uart_baud(0), 115200)
        self.assertEqual(b.ref.ref_baud(), 115200)

    def test_broadcast_without_fast_bit_stays_slow(self):
        b = Lockstep()
        b.run(300)
        b.send(handshake(0x21, 0xFF, baud=0))
        b.run(2)
        self.assertEqual(b.sim.uart_baud(0), 115200)
        self.assertEqual(b.ref.ref_baud(), 115200)

    def test_running_bus_suppresses_announce(self):
        b = Lockstep()
        ctrl = srxl2.packet(0xCD, bytes([0x00, 0x00, 0, 0, 0, 0, 0, 0, 0]))   # channel data, no reply wanted
        for _ in range(40):                         # a master already running when the device starts
            b.send(ctrl)
            b.run(11)
        self.assertEqual(b.log['mubl'], [])
        self.assertEqual(b.log['ref'], [])

    def test_silent_master_restarts_handshake(self):
        b = Lockstep()
        ctrl = srxl2.packet(0xCD, bytes([0x00, 0x00, 0, 0, 0, 0, 0, 0, 0]))
        for _ in range(10):
            b.send(ctrl)
            b.run(11)
        t = b.t
        b.run(200)                                  # the master stops (brown-out): both announce again
        mubl, ref = b.since('mubl', t), b.since('ref', t)
        self.assertTrue(mubl and ref)
        self.assertEqual(mubl[0], ref[0])
        first = {who: min(w for w, _ in b.log[who] if w > t) for who in b.log}
        self.assertLessEqual(abs(first['mubl'] - first['ref']), 2, first)

    def test_unit_id_not_zero_does_not_announce(self):
        b = Lockstep(device=0x41)
        b.sim.fresh()
        b.sim.write_handoff(flags=1, srxl2_id=0x41)
        b.sim.boot(keep_ram=True)
        b.run(400)
        self.assertEqual(b.log['mubl'], [])
        self.assertEqual(b.log['ref'], [])

    def test_bad_crc_ignored(self):
        b = Lockstep()
        b.run(300)
        t = b.t
        p = bytearray(handshake(0x21, DEVICE))
        p[-1] ^= 1
        b.send(bytes(p))
        b.run(5)
        replies = {who: [q for q in b.since(who, t) if q[4] != 0] for who in b.log}   # ignore announcements
        self.assertEqual(replies, {'mubl': [], 'ref': []})


class Update(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(1, SRXL2_ONLY).fresh()
        self.t = SimSrxl2Transport(self.sim, 0)
        self.u = srxl2.Updater(self.t, DEVICE, log=lambda *a: None)

    def test_update_and_boot(self):
        img = make_image(self.sim, size=30001, seed=31)
        self.sim.boot()
        self.u.handshake()
        info = self.u.update(img, board_id=self.sim.board_id)
        self.assertEqual(info['app_status'], 1)        # empty before
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.u.boot()
        self.assertEqual(self.sim.run(50), STATE_APP)
        self.assertEqual(self.sim.flash(0, len(img)), img)
        self.assertEqual(self.sim.read_handoff()['srxl2_id'], DEVICE)
        self.assertEqual(self.sim.errors(), 0)

    def test_update_at_400k(self):
        img = make_image(self.sim, size=20000, seed=32)
        self.sim.boot()
        self.u.handshake()
        self.u.set_fast()
        self.u.handshake()
        self.assertEqual(self.sim.uart_baud(0), 400000)
        # pauses longer than 50 ms drop the device back to 115200: keep the bus busy, as a master does
        self.u.update(img)
        self.assertEqual(self.sim.app_status(), APP_OK)

    def test_fast_baud_survives_slow_replies(self):
        """Replies take time (the reply delay, 45 bytes on the wire): the device must not mistake its
        own transmission time for the master's silence and drop back to 115200."""
        self.sim.blocking_time(1)
        self.sim.boot()
        self.u.handshake()
        self.u.set_fast()
        self.u.handshake()
        renegotiated = []
        original = self.u._renegotiate
        self.u._renegotiate = lambda: (renegotiated.append(1), original())
        for _ in range(5):
            self.u.info()
            self.assertEqual(self.sim.uart_baud(0), 400000)
        img = make_image(self.sim, size=8000, seed=34)
        self.u.update(img)
        self.assertEqual(self.sim.uart_baud(0), 400000)
        self.assertEqual(renegotiated, [])
        self.assertEqual(self.sim.app_status(), APP_OK)

    def test_info_holds_bootloader(self):
        self.sim.load(make_image(self.sim))
        self.sim.boot()
        info = self.u.info()
        self.assertEqual(info['app_status'], APP_OK)
        self.assertEqual(info['version'], (1, 2))
        self.assertEqual(info['board_id'], self.sim.board_id)
        self.assertEqual(self.sim.stay_reason(), STAY_HOST)
        self.assertEqual(self.sim.run(3000), STATE_BOOTLOADER)

    def test_retry_is_not_executed_twice(self):
        self.sim.boot()
        self.u.info()
        self.assertEqual(self.u.request(srxl2.OP_BEGIN, struct.pack('<I', 1000))[0], OK - 0x10)
        seq = self.u.seq + 1
        write = srxl2.packet(srxl2.TYPE_DFU, bytes([DEVICE, 0x21, srxl2.OP_WRITE, seq & 0xFF]) +
                             struct.pack('<I', 0) + bytes(64))
        replies = []
        for _ in range(2):                          # the same request twice: a lost reply, retried
            self.t.write(write)
            replies += self.u.parser.feed(self.t.read(0.05))
        self.assertEqual(len(replies), 2)
        self.assertEqual(replies[0], replies[1])
        status, nxt = replies[1][7], struct.unpack_from('<I', replies[1], 8)[0]
        self.assertEqual((status, nxt), (0, 64))    # written once

    def test_other_device_ignored(self):
        self.sim.boot()
        other = srxl2.Updater(self.t, 0x60, log=lambda *a: None)
        with self.assertRaises(srxl2.Error):
            other.request(srxl2.OP_INFO, timeout=0.05, retries=2)

    def test_wrong_crc_rejected(self):
        img = make_image(self.sim, size=10000, seed=33)
        self.sim.boot()
        self.u.info()
        self.u.request(srxl2.OP_BEGIN, struct.pack('<I', len(img)), timeout=5)
        for off in range(0, len(img), 64):
            self.u.request(srxl2.OP_WRITE, struct.pack('<I', off) + img[off:off + 64])
        status, d = self.u.request(srxl2.OP_FINISH, struct.pack('<II', len(img), 0x12345678), timeout=5)
        self.assertEqual(status, 5)                 # MUBL_E_IMAGE
        self.assertEqual(self.sim.app_status(), APP_EMPTY)
        self.assertEqual(self.u.request(srxl2.OP_BOOT)[0], 5)

    def test_truncated_begin_rejected(self):
        """BEGIN without its four size bytes is refused before anything is erased."""
        img = make_image(self.sim, size=10000, seed=36)
        self.sim.load(img)
        self.sim.boot()
        self.u.info()
        for n in range(4):
            self.assertEqual(self.u.request(srxl2.OP_BEGIN, bytes(n))[0], 7)      # MUBL_E_ARGUMENT
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.assertEqual(self.sim.flash(0, len(img)), img)
        self.assertEqual(self.u.request(srxl2.OP_WRITE, struct.pack('<I', 0) + bytes(64))[0], 6)   # no update
        self.u.boot()
        self.assertEqual(self.sim.run(50), STATE_APP)

    def test_busy_while_serial_updates(self):
        self.sim = Sim(1, SRXL2_ONLY).fresh()
        self.t = SimSrxl2Transport(self.sim, 0)
        self.u = srxl2.Updater(self.t, DEVICE, log=lambda *a: None)
        self.sim.boot()
        c = SerialClient(self.sim, port=1)
        c.identify()
        self.assertEqual(c.erase(), OK)
        self.assertEqual(self.u.request(srxl2.OP_BEGIN, struct.pack('<I', 100), timeout=1)[0], 1)   # BUSY


class LossyTransport(SimSrxl2Transport):
    """Loses the device's next `drop` answers, as noise on the wire would."""
    drop = 0

    def read(self, timeout):
        out = super().read(timeout)
        if out and self.drop:
            while super().read(0.003):                 # the rest of the packet
                pass
            self.drop -= 1
            return b''
        return out


class LostReplies(unittest.TestCase):
    """mubl_srxl2.Updater at 400000 baud when the device's answer is lost: while the device owns
    an update it keeps the fast rate for two seconds of silence, longer than a request's timeout."""

    def setUp(self):
        self.sim = Sim(1, SRXL2_ONLY).fresh()
        self.t = LossyTransport(self.sim, 0)
        self.u = srxl2.Updater(self.t, DEVICE, log=lambda *a: None)
        self.img = make_image(self.sim, size=4000, seed=35)
        self.sim.boot()
        self.u.handshake()
        self.u.set_fast()
        self.u.handshake()
        self.assertEqual(self.u.request(srxl2.OP_BEGIN, struct.pack('<I', len(self.img)), timeout=5)[0], 0)
        self.renegotiated = 0
        original = self.u._renegotiate

        def counted():
            self.renegotiated += 1
            original()
        self.u._renegotiate = counted

    def write(self, off):
        return self.u.request(srxl2.OP_WRITE, struct.pack('<I', off) + self.img[off:off + 64], timeout=1.0)

    def finish(self):
        for off in range(64, len(self.img), 64):
            self.assertEqual(self.write(off)[0], 0)
        crc = zlib.crc32(self.img) & 0xFFFFFFFF
        self.assertEqual(self.u.request(srxl2.OP_FINISH, struct.pack('<II', len(self.img), crc), timeout=10)[0], 0)
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.assertEqual(self.sim.flash(0, len(self.img)), self.img)
        self.assertEqual(self.sim.errors(), 0)

    def test_lost_answer_repeated_at_400k(self):
        self.t.drop = 1
        status, d = self.write(0)
        self.assertEqual((status, struct.unpack_from('<I', d)[0]), (0, 64))       # written once
        self.assertEqual(self.renegotiated, 0)
        self.assertEqual(self.sim.uart_baud(0), 400000)
        self.finish()

    def test_waits_for_device_to_leave_400k_before_renegotiating(self):
        self.t.drop = 2                                 # the repeat at 400000 is lost too
        status, d = self.write(0)
        self.assertEqual((status, struct.unpack_from('<I', d)[0]), (0, 64))       # still written once
        self.assertEqual(self.renegotiated, 1)
        self.assertEqual(self.sim.uart_baud(0), 400000)
        self.finish()


if __name__ == '__main__':
    unittest.main()


@unittest.skipIf(sys.platform == 'win32', 'needs a pseudo-terminal')
class Tool(unittest.TestCase):
    """tools/mubl_srxl2.py from its command line, over a pseudo-terminal to the simulation in real time."""

    def run_tool(self, *extra):
        import tempfile
        from sim import build_exe
        exe = build_exe('tests/sim_pty.c', defines=SRXL2_ONLY)
        sim = Sim(1, SRXL2_ONLY)
        tmp = tempfile.mkdtemp()
        img = make_image(sim, size=20001, seed=41)
        with open(os.path.join(tmp, 'app.bin'), 'wb') as f:
            f.write(img)
        dump = os.path.join(tmp, 'flash.bin')
        proc = subprocess.Popen([os.path.join(exe), '0', '-', dump], stdout=subprocess.PIPE, text=True)
        try:
            pty = proc.stdout.readline().strip()
            if not pty:
                self.skipTest('no pseudo-terminal available (sandboxed?)')
            env = dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'tests', 'shim'))
            r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'mubl_srxl2.py'), pty,
                                os.path.join(tmp, 'app.bin'), '--no-echo', '--board-id', str(sim.board_id)] + list(extra),
                               env=env, capture_output=True, text=True, timeout=120)
            tail = proc.communicate(timeout=30)[0]
        finally:
            proc.kill()
            proc.stdout.close()
            proc.wait()
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('app %08x' % (sim.app_base + 0x401), tail)
        with open(dump, 'rb') as f:
            self.assertEqual(f.read(len(img)), img)

    def test_update(self):
        self.run_tool()

    def test_update_fast(self):
        self.run_tool('--fast')
