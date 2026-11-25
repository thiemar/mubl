"""The serial link: ArduPilot's uploader.py through a pseudo-terminal, and protocol details."""

import os
import subprocess
import sys
import tempfile
import time
import unittest

from clients import FAILED, INVALID, OK, NoReply, SerialClient
from sim import (APP_OK, ROOT, STATE_APP, STATE_BOOTLOADER, STAY_HOST, Sim, build_exe, make_image, mubl_image)

UPLOADER = os.environ.get('UPLOADER_PY', os.path.join(ROOT, '..', 'ardupilot', 'Tools', 'scripts', 'uploader.py'))


@unittest.skipUnless(os.path.exists(UPLOADER) and sys.platform != 'win32', 'uploader.py not found')
class Uploader(unittest.TestCase):
    """uploader.py, unmodified, flashes the simulated board over a pseudo-terminal."""

    def run_upload(self, port, old_image=None):
        exe = build_exe('tests/sim_pty.c', defines={'MUBL_SERIAL_PORT': port})
        sim = Sim(1)
        tmp = tempfile.mkdtemp()
        img = make_image(sim, size=40003, seed=21)
        apj = os.path.join(tmp, 'app.apj')
        with open(apj, 'w') as f:
            f.write(mubl_image.apj(img, sim.board_id))
        old = '-'
        if old_image is not None:
            old = os.path.join(tmp, 'old.bin')
            with open(old, 'wb') as f:
                f.write(old_image)
        dump = os.path.join(tmp, 'flash.bin')
        proc = subprocess.Popen([exe, str(port), old, dump], stdout=subprocess.PIPE, text=True)
        try:
            pty = proc.stdout.readline().strip()
            if not pty:
                self.skipTest('no pseudo-terminal available (sandboxed?)')
            env = dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'tests', 'shim'))
            up = subprocess.run([sys.executable, UPLOADER, '--port', pty, '--no-extf', apj], env=env,
                                capture_output=True, text=True, timeout=120)
            tail = proc.communicate(timeout=30)[0]
        finally:
            proc.kill()
            proc.stdout.close()
            proc.wait()
        self.assertEqual(up.returncode, 0, up.stdout + up.stderr)
        self.assertIn('app %08x' % (sim.app_base + 0x401), tail)
        with open(dump, 'rb') as f:
            self.assertEqual(f.read(len(img)), img)
        return up.stdout

    def test_upload_shared_port(self):
        """Port 0 is shared with SRXL2, which announces itself at start-up."""
        out = self.run_upload(0)
        self.assertIn('Bootloader Protocol: 5', out)

    def test_upload_dedicated_port(self):
        self.run_upload(1)


class Protocol(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(1, {'MUBL_SERIAL_PORT': 1}).fresh()
        self.c = SerialClient(self.sim, port=1)

    def test_nothing_before_sync(self):
        self.sim.boot()
        with self.assertRaises(NoReply):
            self.c.command([0x22, 1, 0x20], 4, timeout_ms=50)
        self.assertEqual(self.c.sync(), OK)

    def test_erase_needs_identify(self):
        self.sim.boot()
        self.assertEqual(self.c.sync(), OK)
        self.assertEqual(self.c.erase(), INVALID)
        self.c.identify()
        self.assertEqual(self.c.erase(), OK)

    def test_program_needs_erase(self):
        self.sim.boot()
        self.c.identify()
        self.assertEqual(self.c.program(b'\0' * 8), INVALID)

    def test_program_length_multiple_of_four(self):
        self.sim.boot()
        self.c.identify()
        self.c.erase()
        self.assertEqual(self.c.program(b'\0' * 6), INVALID)

    def test_partial_command_times_out(self):
        self.sim.boot()
        self.c.sync()
        self.sim.uart_send(1, bytes([0x22, 1]))         # GET_DEVICE with no EOC
        self.sim.run(600)
        self.assertEqual(self.c.sync(), OK)

    def test_identify_holds_bootloader(self):
        self.sim.load(make_image(self.sim))
        self.sim.boot()
        self.c.identify()
        self.assertEqual(self.sim.stay_reason(), STAY_HOST)
        self.assertEqual(self.sim.run(3000), STATE_BOOTLOADER)
        self.assertEqual(self.c.boot(), OK)
        self.assertEqual(self.sim.run(50), STATE_APP)

    def test_sync_alone_does_not_hold(self):
        self.sim.load(make_image(self.sim))
        self.sim.boot()
        self.c.sync()
        self.assertEqual(self.sim.run(1100), STATE_APP)

    def test_serial_number(self):
        self.sim.boot()
        self.c.sync()
        data, status = self.c.command([0x2B, 4, 0, 0, 0, 0x20], 4)
        self.assertEqual((data, status), (bytes(range(0xA4, 0xA8)), OK))

    def test_set_baud(self):
        self.sim.boot()
        self.c.identify()
        self.assertEqual(self.c.command(bytes([0x33]) + (921600).to_bytes(4, 'little') + b'\x20')[1], OK)
        with self.assertRaises(NoReply):
            self.c.command([0x21, 0x20], timeout_ms=50)
        self.c.baud = 921600
        self.assertEqual(self.c.sync(), OK)

    def test_abandoned_update_is_released(self):
        self.sim.boot()
        self.c.identify()
        self.assertEqual(self.c.erase(), OK)
        self.sim.run(31000)
        # another link may now start; here: the same protocol erases again from scratch
        self.assertEqual(self.c.sync(), OK)
        self.c.identify()
        self.assertEqual(self.c.erase(), OK)

    def test_boot_without_valid_image_fails(self):
        self.sim.boot()
        self.c.sync()
        self.assertEqual(self.c.boot(), FAILED)
        self.assertEqual(self.sim.run(1500), STATE_BOOTLOADER)


if __name__ == '__main__':
    unittest.main()
