"""Boot policy, image validation, handoff and the update engine, on the host simulation."""

import os
import random
import struct
import unittest
import zlib

from clients import OK, FAILED, SerialClient
from sim import (APP_BAD_BOARD, APP_BAD_CRC, APP_BAD_SIZE, APP_BAD_VECTORS, APP_EMPTY, APP_FLASH_ERROR,
                 APP_NO_DESCRIPTOR, APP_OK, HANDOFF_TO_APP, STATE_APP, STATE_BOOTLOADER, STATE_OFF,
                 STAY_BOOT_LOOP, STAY_HOLD_PIN, STAY_NO, STAY_NO_APP, STAY_REQUESTED, Sim, make_image, mubl_image)

BOOT_DELAY = 1000


class Crc(unittest.TestCase):
    def test_check_values(self):
        self.assertEqual(mubl_image.crc64_we(b'123456789'), 0x62EC59E3F1A4F00A)
        self.assertEqual(mubl_image.crc32(b'123456789'), (~zlib.crc32(b'123456789', 0xFFFFFFFF)) & 0xFFFFFFFF)


class Boot(unittest.TestCase):
    geometry = 1
    defines = None

    def setUp(self):
        self.sim = Sim(self.geometry, self.defines).fresh()

    def boot_with(self, image):
        self.sim.load(image)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)

    def test_valid_image_boots_after_delay(self):
        img = make_image(self.sim)
        self.boot_with(img)
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.assertEqual(self.sim.run(BOOT_DELAY - 10), STATE_BOOTLOADER)
        self.assertEqual(self.sim.run(20), STATE_APP)
        self.assertEqual(self.sim.app_sp(), 0x20004000)
        self.assertEqual(self.sim.app_entry(), self.sim.app_base + 0x401)
        h = self.sim.read_handoff()
        self.assertEqual(h['magic'], HANDOFF_TO_APP)
        self.assertTrue(h['crc_ok'])
        self.assertEqual(h['boot_attempts'], 1)
        self.assertEqual(self.sim.errors(), 0)

    def test_legacy_descriptor(self):
        self.boot_with(make_image(self.sim, fmt='legacy'))
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.assertEqual(self.sim.run(BOOT_DELAY + 10), STATE_APP)

    def check_rejected(self, image, status):
        self.boot_with(image)
        self.assertEqual(self.sim.app_status(), status)
        self.assertEqual(self.sim.stay_reason(), STAY_NO_APP)
        self.assertEqual(self.sim.run(3 * BOOT_DELAY), STATE_BOOTLOADER)

    def test_empty(self):
        self.check_rejected(b'', APP_EMPTY)

    def test_wrong_board(self):
        self.check_rejected(make_image(self.sim, board_id=self.sim.board_id + 1), APP_BAD_BOARD)

    def test_corrupt_byte(self):
        img = bytearray(make_image(self.sim, size=30000))
        img[25000] ^= 0x10
        self.check_rejected(img, APP_BAD_CRC)

    def test_corrupt_legacy(self):
        img = bytearray(make_image(self.sim, fmt='legacy'))
        img[100] ^= 1
        self.check_rejected(img, APP_BAD_CRC)

    def test_size_outside_region(self):
        img = bytearray(make_image(self.sim))
        struct.pack_into('<I', img, 0x200 + 16, self.sim.app_size + 4)   # image_size
        self.check_rejected(img, APP_BAD_SIZE)

    def test_size_inside_descriptor(self):
        img = bytearray(make_image(self.sim))
        struct.pack_into('<I', img, 0x200 + 16, 0x200 + 8)
        self.check_rejected(img, APP_BAD_SIZE)

    def test_bad_vectors(self):
        img = bytearray(make_image(self.sim))
        struct.pack_into('<I', img, 0, 0x2000FFFF)     # SP half-programmed on a half-word part
        self.check_rejected(img, APP_BAD_VECTORS)
        img = bytearray(make_image(self.sim))
        struct.pack_into('<I', img, 4, self.sim.app_base - 0x1000 + 1)   # entry in the bootloader
        self.sim.fresh()
        self.check_rejected(img, APP_BAD_VECTORS)

    def test_no_descriptor(self):
        self.check_rejected(make_image(self.sim, fmt='raw'), APP_NO_DESCRIPTOR)

    def test_sp_outside_board_ram(self):
        """Correctly stamped, but the stack is in memory this board does not have."""
        for sp in (0x30000000, 0x24001000, 0x20010004, 0x10002004):
            self.sim.fresh()
            self.check_rejected(make_image(self.sim, sp=sp), APP_BAD_VECTORS)

    def test_sp_in_board_ram(self):
        # the top of main RAM, and the extra RAM the sim board declares (MUBL_STACK_RAM, CCM-like)
        for sp in (0x20010000, 0x20000100, 0x10002000, 0x10000400):
            self.sim.fresh()
            self.boot_with(make_image(self.sim, sp=sp))
            self.assertEqual(self.sim.app_status(), APP_OK, hex(sp))

    def test_entry_outside_image(self):
        """A small valid image whose reset vector points into erased flash beyond it."""
        for fmt in ('ap', 'legacy'):
            self.sim.fresh()
            self.check_rejected(make_image(self.sim, size=2048, fmt=fmt, entry=self.sim.app_base + 0x10001),
                                APP_BAD_VECTORS)
            self.sim.fresh()
            self.check_rejected(make_image(self.sim, size=2048, fmt=fmt, entry=self.sim.app_base + 2048 + 1),
                                APP_BAD_VECTORS)

    def test_entry_at_end_of_image(self):
        self.boot_with(make_image(self.sim, size=2048, entry=self.sim.app_base + 2046 + 1))
        self.assertEqual(self.sim.app_status(), APP_OK)

    def test_hold_pin(self):
        self.sim.set_hold(1)
        self.boot_with(make_image(self.sim))
        self.assertEqual(self.sim.stay_reason(), STAY_HOLD_PIN)
        self.assertEqual(self.sim.run(3 * BOOT_DELAY), STATE_BOOTLOADER)

    def test_handoff_request_is_consumed(self):
        self.sim.load(make_image(self.sim))
        self.sim.write_handoff(flags=1)
        self.assertEqual(self.sim.boot(keep_ram=True), STATE_BOOTLOADER)
        self.assertEqual(self.sim.stay_reason(), STAY_REQUESTED)
        self.assertEqual(self.sim.run(3 * BOOT_DELAY), STATE_BOOTLOADER)
        self.assertNotEqual(self.sim.read_handoff()['magic'], 0x4C42756D)
        # the same RAM after a reset no longer holds a request
        self.assertEqual(self.sim.boot(keep_ram=True), STATE_BOOTLOADER)
        self.assertEqual(self.sim.stay_reason(), STAY_NO)

    def test_corrupt_handoff_ignored(self):
        self.sim.load(make_image(self.sim))
        self.sim.write_handoff(flags=1)
        raw = bytearray(self.sim.ram(self.sim.handoff_addr, 32))
        raw[11] ^= 0                     # flags unchanged
        raw[16] ^= 1                     # bitrate changed: CRC fails
        self.sim.lib.sim_ram_set(self.sim.handoff_addr, bytes(raw), len(raw))
        self.sim.boot(keep_ram=True)
        self.assertEqual(self.sim.stay_reason(), STAY_NO)

    def test_boot_loop(self):
        self.sim.load(make_image(self.sim))
        self.sim.boot()
        for attempt in range(1, 4):
            self.assertEqual(self.sim.run(BOOT_DELAY + 10), STATE_APP, attempt)
            self.assertEqual(self.sim.read_handoff()['boot_attempts'], attempt)
            self.sim.app_reset()         # the application crashes before confirming
        self.assertEqual(self.sim.stay_reason(), STAY_BOOT_LOOP)
        self.assertEqual(self.sim.run(3 * BOOT_DELAY), STATE_BOOTLOADER)

    def test_confirmed_boot_resets_count(self):
        self.sim.load(make_image(self.sim))
        self.sim.boot()
        for _ in range(5):
            self.assertEqual(self.sim.run(BOOT_DELAY + 10), STATE_APP)
            self.sim.lib.sim_ram_set(self.sim.handoff_addr, bytes(4), 4)    # mubl_app_confirm()
            self.sim.app_reset()
        self.assertEqual(self.sim.stay_reason(), STAY_NO)

    def test_power_on_ram_is_not_a_request(self):
        self.sim.load(make_image(self.sim))
        for seed in range(20):
            self.sim.boot()
            self.assertEqual(self.sim.stay_reason(), STAY_NO)


class BootF0(Boot):
    geometry = 0

    def test_no_descriptor(self):
        # this configuration accepts raw images
        self.boot_with(make_image(self.sim, fmt='raw'))
        self.assertEqual(self.sim.app_status(), APP_OK)


class BootH7(Boot):
    geometry = 2

    def test_ecc_error_in_image(self):
        img = make_image(self.sim, size=40000)
        self.sim.load(img)
        self.sim.flash_corrupt(30000)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)
        self.assertEqual(self.sim.app_status(), APP_FLASH_ERROR)
        self.assertEqual(self.sim.errors(), 0)          # read only under hal_guarded
        # an update erases the damage
        SerialClient(self.sim).upload(img, boot=False)
        self.assertEqual(SerialClient(self.sim).boot(), OK)
        self.assertEqual(self.sim.run(50), STATE_APP)
        self.assertEqual(self.sim.errors(), 0)

    def test_vec_area_after_interrupted_update(self):
        """First flash word erased, the next one damaged (an update interrupted while writing it): the
        image check stops at the erased word, and GET_DEVICE(VEC_AREA) must not fault on the rest."""
        self.sim.flash_corrupt(32)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)
        self.assertEqual(self.sim.app_status(), APP_EMPTY)
        c = SerialClient(self.sim)
        c.sync()
        data, status = c.command([0x22, 5, 0x20], 0)
        self.assertEqual(status, FAILED)
        self.assertEqual(self.sim.errors(), 0)
        self.assertEqual(self.sim.state(), STATE_BOOTLOADER)

    def test_ecc_error_in_first_word(self):
        self.sim.load(make_image(self.sim))
        self.sim.flash_corrupt(0)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)
        self.assertEqual(self.sim.app_status(), APP_FLASH_ERROR)
        self.assertEqual(self.sim.errors(), 0)


class BootH7RawImages(BootH7):
    """MUBL_RAW_IMAGES (as tbs-lucid-h7, for stock ArduPilot): an image without a descriptor boots on
    its vectors; one with a descriptor is still checked by it (the inherited tests)."""
    defines = {'MUBL_RAW_IMAGES': 1}

    def test_no_descriptor(self):
        self.boot_with(make_image(self.sim, fmt='raw', size=40000))
        self.assertEqual(self.sim.app_status(), APP_OK)
        self.assertEqual(self.sim.run(BOOT_DELAY + 10), STATE_APP)
        self.assertEqual(self.sim.errors(), 0)

    def test_raw_image_needs_plausible_vectors(self):
        self.check_rejected(make_image(self.sim, fmt='raw', sp=0x30000000), APP_BAD_VECTORS)

    def test_raw_upload_boots(self):
        img = make_image(self.sim, fmt='raw', size=50001, seed=3)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)            # empty flash: stays
        _, status = SerialClient(self.sim).upload(img)
        self.assertEqual(status, OK)
        self.assertEqual(self.sim.run(50), STATE_APP)
        self.assertEqual(self.sim.flash(0, len(img)), img)
        self.assertEqual(self.sim.errors(), 0)


class BootH7CrcUnit(BootH7):
    """As on the hardware: CRC-32 in the CRC unit, whose clock hal_deinit stops (the handoff to the
    application must be written before that)."""
    defines = {'MUBL_HW_CRC32': 1}


class Update(unittest.TestCase):
    geometry = 1
    defines = None
    image_fmt = 'ap'                    # of the images in test_power_loss_anywhere

    def setUp(self):
        self.sim = Sim(self.geometry, self.defines).fresh()

    def test_serial_update_boots(self):
        img = make_image(self.sim, size=50001)
        self.assertEqual(self.sim.boot(), STATE_BOOTLOADER)
        c = SerialClient(self.sim)
        info, status = c.upload(img)
        self.assertEqual(info, {'bl_rev': 5, 'board_id': self.sim.board_id, 'board_rev': 0,
                                'fw_size': self.sim.app_size})
        self.assertEqual(status, OK)
        self.assertEqual(self.sim.run(50), STATE_APP)
        padded = img + b'\xff' * (-len(img) % 4)
        self.assertEqual(self.sim.flash(0, len(padded)), padded)
        self.assertEqual(self.sim.errors(), 0)

    def test_tail_of_larger_image_erased(self):
        big = make_image(self.sim, size=150000, seed=1)
        self.sim.load(big)
        self.sim.write_handoff(flags=1)
        self.sim.boot(keep_ram=True)
        small = make_image(self.sim, size=10000, seed=2)
        _, status = SerialClient(self.sim).upload(small)
        self.assertEqual(status, OK)
        self.assertEqual(self.sim.flash(len(small), 150000 - len(small)), b'\xff' * (150000 - len(small)))

    def test_crc_matches_uploader(self):
        img = make_image(self.sim, size=20000)
        self.sim.boot()
        c = SerialClient(self.sim)
        c.upload(img, boot=False)
        # uploader.py: CRC over the image padded with 0xFF to the region size, before BOOT
        expect = mubl_image.crc32(img + b'\xff' * (self.sim.app_size - len(img)))
        self.assertEqual(c.crc(), expect)
        self.assertEqual(self.sim.app_status(), APP_EMPTY)    # not bootable until BOOT
        self.assertEqual(c.boot(), OK)

    def test_rejected_image_stays(self):
        img = make_image(self.sim, board_id=1)
        self.sim.boot()
        c = SerialClient(self.sim)
        c.upload(img, boot=False)
        self.assertEqual(c.boot(), FAILED)
        self.assertEqual(self.sim.app_status(), APP_BAD_BOARD)
        self.assertEqual(self.sim.run(3 * BOOT_DELAY), STATE_BOOTLOADER)
        self.assertEqual(self.sim.flash(0, 4), b'\xff' * 4)   # first word never written

    def test_power_loss_anywhere(self):
        """Power fails at every flash operation of an update over a previous image: afterwards the
        bootloader never starts a partial image, and a new update always succeeds."""
        old = make_image(self.sim, size=min(60000, self.sim.app_size * 6 // 10), seed=10, version=(1, 0),
                         fmt=self.image_fmt)
        new = make_image(self.sim, size=min(45001, self.sim.app_size * 45 // 100), seed=11, version=(2, 0),
                         fmt=self.image_fmt)
        # count the operations of a full update
        self.sim.fresh()
        self.sim.load(old)
        self.sim.write_handoff(flags=1)
        self.sim.boot(keep_ram=True)
        start = self.sim.flash_ops()
        SerialClient(self.sim).upload(new)
        total = self.sim.flash_ops() - start
        self.assertGreater(total, 10)
        points = list(range(1, total + 1))
        if len(points) > 400 and not os.environ.get('MUBL_EXHAUSTIVE'):      # every point: MUBL_EXHAUSTIVE=1
            rng = random.Random(1)
            points = sorted(set(points[:50] + points[-50:] + rng.sample(points, 300)))
        padded = new + b'\xff' * (-len(new) % 4)
        for k in points:
            self.sim.fresh()
            self.sim.load(old)
            self.sim.write_handoff(flags=1)
            self.sim.boot(keep_ram=True)
            self.sim.inject_powerloss(k, k)
            try:
                SerialClient(self.sim).upload(new)
            except Exception:
                pass
            self.assertEqual(self.sim.state(), STATE_OFF, k)
            self.assertEqual(self.sim.errors(), 0, k)
            state = self.sim.boot()
            if state == STATE_BOOTLOADER:
                state = self.sim.run(BOOT_DELAY + 10)
            if state == STATE_APP:
                # only a complete image may start
                self.assertIn(self.sim.flash(0, len(padded)), (padded, old[:len(padded)]), k)
            self.assertEqual(self.sim.errors(), 0, k)
            if state != STATE_BOOTLOADER:
                self.sim.write_handoff(flags=1)
                self.sim.boot(keep_ram=True)
            _, status = SerialClient(self.sim).upload(new)
            self.assertEqual(status, OK, k)
            self.assertEqual(self.sim.run(50), STATE_APP, k)
            self.assertEqual(self.sim.flash(0, len(padded)), padded, k)
            self.assertEqual(self.sim.errors(), 0, k)


class UpdateF0(Update):
    geometry = 0

    def test_tail_of_larger_image_erased(self):
        big = make_image(self.sim, size=50000, seed=1)
        self.sim.load(big)
        self.sim.write_handoff(flags=1)
        self.sim.boot(keep_ram=True)
        small = make_image(self.sim, size=10000, seed=2)
        _, status = SerialClient(self.sim).upload(small)
        self.assertEqual(status, OK)
        self.assertEqual(self.sim.flash(len(small), 40000), b'\xff' * 40000)


class UpdateH7(Update):
    geometry = 2


class UpdateH7RawImages(UpdateH7):
    """Raw images have no CRC: only writing the first block last keeps a partial one from starting."""
    defines = {'MUBL_RAW_IMAGES': 1}
    image_fmt = 'raw'


class UpdateH7CrcUnit(UpdateH7):
    defines = {'MUBL_HW_CRC32': 1}


if __name__ == '__main__':
    unittest.main()


class AppHeader(unittest.TestCase):
    """app/mubl_app.h, as an application uses it, against the simulated bootloader."""
    defines = None

    def setUp(self):
        import ctypes
        import subprocess
        from sim import BUILD, CC, ROOT
        out = os.path.join(BUILD, 'libappapi.so')
        src = os.path.join(ROOT, 'tests', 'app_api.c')
        hdr = os.path.join(ROOT, 'app', 'mubl_app.h')
        if not os.path.exists(out) or os.path.getmtime(out) < max(os.path.getmtime(src), os.path.getmtime(hdr)):
            os.makedirs(BUILD, exist_ok=True)
            subprocess.check_call([CC, '-std=c11', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                                   '-I' + os.path.join(ROOT, 'app'), src, '-o', out])

        class Info(ctypes.Structure):
            _fields_ = [('link', ctypes.c_uint8), ('boot_attempts', ctypes.c_uint8), ('can_node_id', ctypes.c_uint8),
                        ('srxl2_device_id', ctypes.c_uint8), ('can_bitrate', ctypes.c_uint32),
                        ('uart_baud', ctypes.c_uint32)]
        self.Info = Info
        self.api = ctypes.CDLL(out)
        self.api.app_block.restype = ctypes.POINTER(ctypes.c_uint8 * 256)
        self.api.app_request.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint8,
                                         ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint8]
        self.api.app_load.argtypes = [ctypes.c_char_p]
        self.api.app_info.argtypes = [ctypes.POINTER(Info)]
        self.sim = Sim(1, self.defines).fresh()

    def block(self):
        return bytes(self.api.app_block().contents)

    def test_request_and_info(self):
        import ctypes
        self.sim.load(make_image(self.sim))
        self.api.app_request(2, 0, 0, 0, None, 230400, 0)            # serial, at 230400
        self.sim.lib.sim_ram_set(self.sim.handoff_addr, self.block(), 256)
        self.sim.boot(keep_ram=True)
        self.assertEqual(self.sim.stay_reason(), 2)                    # MUBL_STAY_REQUESTED
        c = SerialClient(self.sim, 0, baud=230400)
        c.identify()
        self.assertEqual(c.boot(), OK)
        self.assertEqual(self.sim.run(50), STATE_APP)
        self.api.app_load(self.sim.ram(self.sim.handoff_addr, 256))
        info = self.Info()
        self.assertEqual(self.api.app_info(ctypes.byref(info)), 1)
        self.assertEqual((info.link, info.boot_attempts, info.uart_baud), (2, 1, 230400))
        self.assertEqual(self.sim.errors(), 0)
        self.api.app_confirm()
        self.assertEqual(self.api.app_info(ctypes.byref(info)), 0)

    def test_dronecan_request_with_path(self):
        self.api.app_request(1, 42, 500000, 127, b'fw/app.bin', 0, 0)
        self.sim.lib.sim_ram_set(self.sim.handoff_addr, self.block(), 256)
        self.sim.boot(keep_ram=True)
        self.assertEqual(self.sim.can_bitrate(), 500000)
        self.assertEqual(self.sim.node_id(), 42)


class AppHeaderCrcUnit(AppHeader):
    defines = {'MUBL_HW_CRC32': 1}


class Divide(unittest.TestCase):
    """src/hal/stm32/divide.c (the Cortex-M0 division helpers) against the host's division."""

    def test_against_native(self):
        import subprocess
        from sim import BUILD, CC, ROOT
        out = os.path.join(BUILD, 'divide_test')
        os.makedirs(BUILD, exist_ok=True)
        subprocess.check_call([CC, '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                               os.path.join(ROOT, 'tests', 'divide_test.c'), '-o', out])
        r = subprocess.run([out], capture_output=True, text=True)
        self.assertEqual((r.returncode, r.stdout.strip()), (0, 'ok'), r.stdout)


class HardwareCrc(unittest.TestCase):
    """src/hal/stm32/crc_hw.c (CRC-32 in the CRC unit) against a model of the unit."""

    def test_against_model(self):
        import subprocess
        from sim import BUILD, CC, ROOT
        out = os.path.join(BUILD, 'crc_hw_test')
        os.makedirs(BUILD, exist_ok=True)
        subprocess.check_call([CC, '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-DMUBL_CRC_MODEL',
                               '-DMUBL_HW_CRC32=1', '-I' + os.path.join(ROOT, 'boards/sim'),
                               '-I' + os.path.join(ROOT, 'src'), '-I' + os.path.join(ROOT, 'src/dronecan/classic'),
                               os.path.join(ROOT, 'tests', 'crc_hw_test.c'),
                               os.path.join(ROOT, 'src', 'hal', 'stm32', 'crc_hw.c'),
                               os.path.join(ROOT, 'src', 'hal', 'host', 'crc_model.c'), '-o', out])
        r = subprocess.run([out], capture_output=True, text=True)
        self.assertEqual((r.returncode, r.stdout.strip()), (0, 'ok'), r.stdout)
