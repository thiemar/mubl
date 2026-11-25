"""The USB link: the serial protocol over CDC-ACM, the stay-while-connected rule, and the drivers.

The simulation runs src/hal/usb/usb_fs.c against a model of the F0/F3 USB peripheral, or
usb_otg.c against a model of the F4/H7 OTG core (MUBL_USB_OTG_CORE), with a simulated host that
enumerates the device and moves bulk data (src/hal/host/usb_host.c).  Every scenario runs on the
FS peripheral, the OTG core with the B-session override (F446, H7) and the F405's OTG core.
"""

import os
import subprocess
import sys
import tempfile
import unittest

from clients import FAILED, OK, NoReply, SerialClient, UsbSerialClient
from sim import (BUILD, CC, LINK_SERIAL, LINK_USB, ROOT, STATE_APP, STATE_BOOTLOADER, STAY_HOST, STAY_NO,
                 STAY_REQUESTED, STAY_USB, Sim, build_exe, make_image, mubl_image)
from test_serial import UPLOADER

DETACHED, DEBOUNCE, CONFIGURED = 0, 1, 2


def boot_with_host(sim, image=None, attach_ms=100):
    if image is not None:
        sim.load(image)
    sim.usb_attach_ms(attach_ms)
    sim.usb_plug(1)
    sim.boot()


FS, OTG, OTG_F405 = {}, {'MUBL_USB_OTG_CORE': 1}, {'MUBL_USB_OTG_CORE': 1, 'STM32_OTG_VBDEN': 0}


class Driver(unittest.TestCase):
    """µCAN's unit tests for its USB driver (tests/usb_fs_test.c), against µBL's port of it and,
    without the four that probe FS register races, against the OTG driver."""

    def run_driver_test(self, otg):
        out = os.path.join(BUILD, 'usb_driver_test_%d' % otg)
        srcs = [os.path.join(ROOT, p) for p in ('src/hal/usb/usb_fs.c', 'src/hal/usb/usb_otg.c', 'src/hal/usb/usb_cdc.c',
                                                'src/hal/host/usb_fs_model.c', 'src/hal/host/usb_otg_model.c',
                                                'tests/usb_fs_test.c')]
        flags = ['-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-DMUBL_USB_OTG_CORE=%d' % otg,
                 '-I' + os.path.join(ROOT, 'boards/sim'),
                 '-I' + os.path.join(ROOT, 'src'), '-I' + os.path.join(ROOT, 'src/dronecan/classic'),
                 '-DMUBL_USB=1', '-DMUBL_USB_TX_SIZE=1024', '-DMUBL_USB_MANUFACTURER="Thiemar"',
                 '-DMUBL_USB_PRODUCT="muCAN"', '-DMUBL_USB_BCD_DEVICE=0x0200']
        os.makedirs(BUILD, exist_ok=True)
        try:
            subprocess.check_call([CC] + flags + ['-fsanitize=address,undefined'] + srcs + ['-o', out],
                                  stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            subprocess.check_call([CC] + flags + srcs + ['-o', out])
        r = subprocess.run([out], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn(' 0 failed', r.stdout)

    def test_fs_driver(self):
        self.run_driver_test(0)

    def test_otg_driver(self):
        self.run_driver_test(1)


class Attach:
    DEFINES = FS

    def setUp(self):
        self.sim = Sim(1, self.DEFINES).fresh()
        self.img = make_image(self.sim, size=20000, seed=7)

    def tearDown(self):
        self.assertEqual(self.sim.usb_model_errors(), 0)
        self.assertEqual(self.sim.errors(), 0)

    def test_descriptors(self):
        boot_with_host(self.sim)
        self.sim.run(200)
        self.assertEqual(self.sim.usb_state(), CONFIGURED)
        dev = self.sim.usb_descriptor(0)
        self.assertEqual(dev[:2], bytes([18, 1]))
        self.assertEqual(dev[4], 2)                                  # CDC at device level
        self.assertEqual(dev[8:12], bytes([0x83, 0x04, 0x40, 0x57]))  # 0483:5740
        cfg = self.sim.usb_descriptor(1)
        self.assertEqual(cfg[2] | cfg[3] << 8, 67)
        serial = self.sim.usb_descriptor(2)
        self.assertEqual(serial[0], 50)
        self.assertEqual(serial[2:50:2].decode(), ''.join('%02X' % (0xA0 + i) for i in range(12)))
        self.assertEqual(self.sim.usb_failures(), 0)

    def test_no_cable_boots_after_delay(self):
        self.sim.load(self.img)
        self.sim.boot()
        self.assertEqual(self.sim.usb_connected(), 1)               # attached, but nobody is there
        self.assertEqual(self.sim.run(990), STATE_BOOTLOADER)
        self.assertEqual(self.sim.run(20), STATE_APP)
        self.assertEqual(self.sim.usb_connected(), 0)

    def test_host_keeps_bootloader(self):
        boot_with_host(self.sim, self.img)
        self.assertEqual(self.sim.run(5000), STATE_BOOTLOADER)
        self.assertEqual(self.sim.stay_reason(), STAY_USB)
        self.assertEqual(self.sim.usb_enumerations(), 1)

    def test_unplugged_host_boots(self):
        boot_with_host(self.sim, self.img)
        self.sim.run(2000)
        self.sim.usb_plug(0)
        self.assertEqual(self.sim.run(480), STATE_BOOTLOADER)
        self.assertEqual(self.sim.run(40), STATE_APP)
        self.assertEqual(self.sim.read_handoff()['link'], LINK_USB)

    def test_suspended_host_boots(self):
        boot_with_host(self.sim, self.img)
        self.sim.run(1500)
        self.sim.usb_suspend(1)
        self.assertEqual(self.sim.run(480), STATE_BOOTLOADER)
        self.assertEqual(self.sim.run(40), STATE_APP)

    def test_host_within_boot_delay(self):
        boot_with_host(self.sim, self.img, attach_ms=900)
        self.assertEqual(self.sim.run(3000), STATE_BOOTLOADER)
        self.assertEqual(self.sim.stay_reason(), STAY_USB)

    def test_host_after_boot_delay(self):
        boot_with_host(self.sim, self.img, attach_ms=1200)
        self.assertEqual(self.sim.run(1100), STATE_APP)
        self.assertEqual(self.sim.usb_enumerations(), 0)

    def test_no_application_stays_without_host(self):
        self.sim.boot()
        self.sim.run(3000)
        self.assertEqual(self.sim.state(), STATE_BOOTLOADER)

    def test_usb_stay_off(self):
        sim = Sim(1, dict(self.DEFINES, MUBL_USB_STAY=0)).fresh()
        boot_with_host(sim, self.img)
        self.assertEqual(sim.run(1100), STATE_APP)
        sim.fresh()
        boot_with_host(sim, self.img)
        sim.run(300)
        UsbSerialClient(sim).identify()
        self.assertEqual(sim.stay_reason(), STAY_HOST)
        self.assertEqual(sim.run(2000), STATE_BOOTLOADER)
        self.assertEqual(sim.usb_model_errors(), 0)

    def test_replug_enumerates_again(self):
        boot_with_host(self.sim)
        self.sim.run(300)
        c = UsbSerialClient(self.sim)
        self.assertEqual(c.sync(), OK)
        self.sim.usb_send([0x22])                  # half a command, then the cable goes
        self.sim.run(10)
        self.sim.usb_plug(0)
        self.sim.run(50)
        self.sim.usb_plug(1)
        self.sim.run(300)
        self.assertEqual(self.sim.usb_enumerations(), 2)
        self.assertEqual(c.identify()['board_id'], self.sim.board_id)


class Update:
    DEFINES = FS

    def check_upload(self, geometry, size, blocking=False):
        sim = Sim(geometry, self.DEFINES).fresh()
        sim.blocking_time(1 if blocking else 0)
        img = make_image(sim, size=size, seed=geometry + 30)
        boot_with_host(sim)
        sim.run(300)
        info, status = UsbSerialClient(sim).upload(img)
        self.assertEqual(info['board_id'], sim.board_id)
        self.assertEqual(status, OK)
        self.assertEqual(sim.run(100), STATE_APP)
        self.assertEqual(sim.flash(0, len(img)), img)
        self.assertEqual(sim.app_entry(), sim.app_base + 0x401)
        h = sim.read_handoff()
        self.assertTrue(h['crc_ok'])
        self.assertEqual(h['link'], LINK_USB)
        self.assertEqual(sim.usb_state(), DETACHED)               # the device left the bus first
        self.assertEqual(sim.usb_detaches(), 1)
        self.assertEqual((sim.usb_model_errors(), sim.errors(), sim.usb_failures()), (0, 0, 0))

    def test_upload_f0(self):
        self.check_upload(0, 30000)

    def test_upload_f4(self):
        self.check_upload(1, 70000)

    def test_upload_h7(self):
        self.check_upload(2, 150000)

    def test_upload_with_blocking_flash(self):
        """Erases and programs take their real time: the host is held off (NAK) meanwhile."""
        self.check_upload(1, 70000, blocking=True)

    def test_crc_matches(self):
        sim = Sim(1, self.DEFINES).fresh()
        img = make_image(sim, size=5000, seed=3)
        boot_with_host(sim)
        sim.run(300)
        c = UsbSerialClient(sim)
        c.upload(img, boot=False)
        expected = mubl_image.crc32(img + b'\xff' * (sim.app_size - len(img)))
        self.assertEqual(c.crc(), expected)

    def test_set_baud_ignored(self):
        sim = Sim(1, self.DEFINES).fresh()
        boot_with_host(sim)
        sim.run(300)
        c = UsbSerialClient(sim)
        c.identify()
        self.assertEqual(c.command(bytes([0x33]) + (921600).to_bytes(4, 'little') + b'\x20')[1], OK)
        self.assertEqual(c.sync(), OK)
        self.assertEqual(sim.uart_baud(0), 115200)

    def test_handoff_request_starts_usb_only(self):
        sim = Sim(1, self.DEFINES).fresh()
        sim.load(make_image(sim))
        sim.write_handoff(flags=1, link=LINK_USB)
        sim.usb_plug(1)
        sim.boot(keep_ram=True)
        self.assertEqual(sim.can_bitrate(), 0)
        self.assertEqual(sim.uart_baud(0), 0)
        self.assertEqual(sim.uart_baud(1), 0)
        sim.run(300)
        self.assertEqual(sim.stay_reason(), STAY_REQUESTED)
        self.assertEqual(UsbSerialClient(sim).sync(), OK)

    def test_one_update_at_a_time(self):
        """The UART and USB instances parse independently; only one owns the update."""
        sim = Sim(1, dict(self.DEFINES, MUBL_SERIAL_PORT=1)).fresh()
        boot_with_host(sim)
        sim.run(300)
        uart, usb = SerialClient(sim, port=1), UsbSerialClient(sim)
        sim.uart_send(1, [0x22, 1])                # half a UART command
        usb.identify()
        sim.uart_send(1, [0x20])                   # completes it: dropped, the port is not claimed yet
        uart.identify()
        self.assertEqual(uart.erase(), OK)
        self.assertEqual(usb.erase(), FAILED)
        self.assertEqual(uart.program(b'\x00' * 16), OK)
        self.assertEqual(usb.program(b'\x00' * 16), 0x13)     # INVALID: not this link's update

    def test_reset_reenumerates(self):
        """An application asking for the bootloader over USB resets; the host sees a new device."""
        sim = Sim(1, self.DEFINES).fresh()
        boot_with_host(sim, make_image(sim))
        sim.run(300)
        c = UsbSerialClient(sim)
        self.assertEqual(c.sync(), OK)
        self.assertEqual(c.boot(), OK)
        self.assertEqual(sim.run(100), STATE_APP)
        sim.write_handoff(flags=1, link=LINK_USB)
        sim.app_reset()
        sim.run(300)
        self.assertEqual(sim.usb_enumerations(), 2)
        self.assertEqual(sim.stay_reason(), STAY_REQUESTED)
        self.assertEqual(sim.usb_model_errors(), 0)


class AttachFs(Attach, unittest.TestCase):
    DEFINES = FS


class AttachOtg(Attach, unittest.TestCase):
    DEFINES = OTG


class AttachOtgF405(Attach, unittest.TestCase):
    DEFINES = OTG_F405


class UpdateFs(Update, unittest.TestCase):
    DEFINES = FS


class UpdateOtg(Update, unittest.TestCase):
    DEFINES = OTG


class UpdateOtgF405(Update, unittest.TestCase):
    DEFINES = OTG_F405


@unittest.skipUnless(os.path.exists(UPLOADER) and sys.platform != 'win32', 'uploader.py not found')
class Uploader(unittest.TestCase):
    """uploader.py, unmodified, flashes the simulated board through its USB port."""

    def run_upload(self, defines):
        exe = build_exe('tests/sim_pty.c', defines=defines)
        sim = Sim(1, defines)
        tmp = tempfile.mkdtemp()
        img = make_image(sim, size=40003, seed=22)
        apj = os.path.join(tmp, 'app.apj')
        with open(apj, 'w') as f:
            f.write(mubl_image.apj(img, sim.board_id))
        dump = os.path.join(tmp, 'flash.bin')
        proc = subprocess.Popen([exe, 'usb', '-', dump], stdout=subprocess.PIPE, text=True)
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

    def test_upload_fs(self):
        self.run_upload(FS)

    def test_upload_otg(self):
        self.run_upload(OTG)


if __name__ == '__main__':
    unittest.main()
