"""The µBL host simulation, built on demand and driven through ctypes.

    sim = Sim(geometry=1, defines={'MUBL_LINK_EXCLUSIVE': 1})

Each distinct set of defines is compiled once into build/sim/ (the library has global state, so a
test process uses one instance per configuration; Sim.fresh() resets it).
"""

import ctypes
import hashlib
import os
import random
import struct
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tools'))

import mubl_image  # noqa: E402

BUILD = os.path.join(ROOT, 'build', 'sim')
CC = os.environ.get('CC', 'cc')

STATE_OFF, STATE_BOOTLOADER, STATE_APP = 0, 1, 2
LINK_DRONECAN, LINK_SERIAL, LINK_SRXL2, LINK_USB = 1, 2, 3, 4
APP_OK, APP_EMPTY, APP_BAD_VECTORS, APP_NO_DESCRIPTOR, APP_BAD_SIZE, APP_BAD_CRC, APP_BAD_BOARD, APP_FLASH_ERROR = range(8)
STAY_NO, STAY_NO_APP, STAY_REQUESTED, STAY_HOLD_PIN, STAY_HOST, STAY_BOOT_LOOP, STAY_USB = range(7)
HANDOFF_TO_BL, HANDOFF_TO_APP = 0x4C42756D, 0x5041756D

SOURCES = ['src/main.c', 'src/update.c', 'src/image.c', 'src/handoff.c', 'src/crc.c', 'src/util.c',
           'src/uart_mux.c', 'src/proto_dronecan.c', 'src/proto_serial.c', 'src/proto_srxl2.c',
           'src/hal/host/hal_host.c', 'src/hal/usb/usb_cdc.c', 'src/hal/usb/usb_fs.c', 'src/hal/usb/usb_otg.c',
           'src/hal/host/usb_fs_model.c', 'src/hal/host/usb_otg_model.c', 'src/hal/host/usb_host.c']
HEADERS = ['src/mubl.h', 'src/mubl_config.h', 'src/mubl_defs.h', 'boards/sim/board.h', 'src/hal/hal.h',
           'src/links.h', 'src/hal/usb/usb_cdc.h', 'src/hal/usb/usb_fs_regs.h', 'src/hal/usb/usb_otg_regs.h',
           'src/hal/host/usb_model.h', 'src/hal/host/usb_fs_model.h', 'src/hal/host/usb_otg_model.h',
           'src/hal/host/usb_host.h']


def build(geometry=1, defines=None, frame='classic'):
    defines = dict(defines or {})
    flags = ['-DSIM_GEOMETRY=%d' % geometry] + ['-D%s=%s' % kv for kv in sorted(defines.items())]
    sources = SOURCES + ['src/dronecan/%s/dronecan.c' % frame]
    digest = hashlib.sha1(' '.join(flags + [frame]).encode()).hexdigest()[:10]
    stamp = max(os.path.getmtime(os.path.join(ROOT, p)) for p in sources + HEADERS)
    out = os.path.join(BUILD, 'libmubl_%s.so' % digest)
    if not os.path.exists(out) or os.path.getmtime(out) < stamp:
        os.makedirs(BUILD, exist_ok=True)
        # (Sanitizers: tests/fuzz.c runs the same sources as a program; a sanitized library loaded
        # into an uninstrumented Python does not work everywhere.)
        cmd = [CC, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
               '-I' + os.path.join(ROOT, 'boards/sim'), '-I' + os.path.join(ROOT, 'src'),
               '-I' + os.path.join(ROOT, 'src/dronecan', frame)] + flags + \
              [os.path.join(ROOT, p) for p in sources] + ['-o', out]
        subprocess.check_call(cmd)
    return out


def build_exe(main, geometry=1, defines=None, extra=()):
    """An executable of the simulation plus `main` (e.g. tests/sim_pty.c)."""
    lib_flags = ['-DSIM_GEOMETRY=%d' % geometry] + ['-D%s=%s' % kv for kv in sorted((defines or {}).items())]
    sources = SOURCES + ['src/dronecan/classic/dronecan.c', main] + list(extra)
    digest = hashlib.sha1(' '.join(lib_flags + sources).encode()).hexdigest()[:10]
    out = os.path.join(BUILD, os.path.splitext(os.path.basename(main))[0] + '_' + digest)
    stamp = max(os.path.getmtime(os.path.join(ROOT, p)) for p in sources + HEADERS)
    if not os.path.exists(out) or os.path.getmtime(out) < stamp:
        os.makedirs(BUILD, exist_ok=True)
        subprocess.check_call([CC, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                               '-I' + os.path.join(ROOT, 'boards/sim'), '-I' + os.path.join(ROOT, 'src'),
                               '-I' + os.path.join(ROOT, 'src/dronecan/classic')] + lib_flags +
                              [os.path.join(ROOT, p) for p in sources] + ['-o', out])
    return out


class Sim:
    def __init__(self, geometry=1, defines=None, frame='classic'):
        self.path = build(geometry, defines, frame)
        lib = self.lib = ctypes.CDLL(self.path)
        u8p = ctypes.POINTER(ctypes.c_uint8)
        sig = {
            'sim_reset_all': ([], None), 'sim_set_uid': ([ctypes.c_char_p], None),
            'sim_set_hold': ([ctypes.c_int], None), 'sim_app_emulation': ([ctypes.c_int], None),
            'sim_blocking_time': ([ctypes.c_int], None),
            'sim_flash_load': ([ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint32], None),
            'sim_flash_get': ([ctypes.c_uint32, u8p, ctypes.c_uint32], None),
            'sim_flash_corrupt': ([ctypes.c_uint32], None), 'sim_flash_is_corrupt': ([ctypes.c_uint32], ctypes.c_int),
            'sim_flash_ops': ([], ctypes.c_uint32),
            'sim_inject_powerloss': ([ctypes.c_uint32, ctypes.c_uint32], None),
            'sim_ram_set': ([ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint32], None),
            'sim_ram_get': ([ctypes.c_uint32, u8p, ctypes.c_uint32], None),
            'sim_power_on': ([ctypes.c_int], None), 'sim_app_reset': ([], None),
            'sim_run': ([ctypes.c_uint32], ctypes.c_int), 'sim_state': ([], ctypes.c_int),
            'sim_now': ([], ctypes.c_uint32), 'sim_set_time': ([ctypes.c_uint32], None),
            'sim_boots': ([], ctypes.c_uint32),
            'sim_app_entry': ([], ctypes.c_uint32), 'sim_app_sp': ([], ctypes.c_uint32),
            'sim_errors': ([], ctypes.c_int), 'sim_led': ([], ctypes.c_int),
            'sim_app_status': ([], ctypes.c_int), 'sim_app_format': ([], ctypes.c_int),
            'sim_stay_reason': ([], ctypes.c_int), 'sim_node_id': ([], ctypes.c_uint8),
            'sim_px4_get': ([ctypes.c_int], ctypes.c_uint32),
            'sim_px4_set': ([ctypes.POINTER(ctypes.c_uint32)], None),
            'sim_app_base': ([], ctypes.c_uint32), 'sim_app_size': ([], ctypes.c_uint32),
            'sim_granule': ([], ctypes.c_uint32), 'sim_handoff_addr': ([], ctypes.c_uint32),
            'sim_ap_comms_addr': ([], ctypes.c_uint32), 'sim_board_id': ([], ctypes.c_uint32),
        }
        if hasattr(lib, 'sim_uart_send'):
            sig.update({
                'sim_uart_send': ([ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_uint32], None),
                'sim_uart_recv': ([ctypes.c_int, u8p, ctypes.c_int, ctypes.c_uint32], ctypes.c_int),
                'sim_uart_baud': ([ctypes.c_int], ctypes.c_uint32),
            })
        if hasattr(lib, 'sim_usb_plug'):
            sig.update({
                'sim_usb_plug': ([ctypes.c_int], None), 'sim_usb_suspend': ([ctypes.c_int], None),
                'sim_usb_attach_ms': ([ctypes.c_uint32], None), 'sim_usb_state': ([], ctypes.c_int),
                'sim_usb_connected': ([], ctypes.c_int), 'sim_usb_enumerations': ([], ctypes.c_uint32),
                'sim_usb_failures': ([], ctypes.c_uint32), 'sim_usb_detaches': ([], ctypes.c_uint32),
                'sim_usb_model_errors': ([], ctypes.c_int), 'sim_usb_descriptor': ([ctypes.c_int, u8p], None),
                'sim_usb_send': ([ctypes.c_char_p, ctypes.c_int], None),
                'sim_usb_recv': ([u8p, ctypes.c_int], ctypes.c_int), 'sim_usb_pending': ([], ctypes.c_int),
            })
        if hasattr(lib, 'sim_can_bus_bitrate'):
            sig.update({
                'sim_can_bus_bitrate': ([ctypes.c_uint32], None), 'sim_can_bitrate': ([], ctypes.c_uint32),
                'sim_can_listen_only': ([], ctypes.c_int),
                'sim_rx': ([ctypes.c_uint32, ctypes.c_char_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint64],
                           ctypes.c_int),
                'sim_tx': ([ctypes.POINTER(ctypes.c_uint32), u8p, u8p], ctypes.c_int),
                'sim_tick': ([ctypes.c_uint64], None),
            })
        for name, (args, res) in sig.items():
            fn = getattr(lib, name)
            fn.argtypes = args
            fn.restype = res
        self.app_base = lib.sim_app_base()
        self.app_size = lib.sim_app_size()
        self.granule = lib.sim_granule()
        self.handoff_addr = lib.sim_handoff_addr()
        self.board_id = lib.sim_board_id()
        self.fresh()

    def fresh(self):
        self.lib.sim_reset_all()
        if os.environ.get('MUBL_SIM_BLOCKING'):      # time passes in blocking calls, in every test
            self.lib.sim_blocking_time(1)
        return self

    # ---- convenience ----
    def __getattr__(self, name):
        return getattr(self.lib, 'sim_' + name)

    def flash(self, offset, n):
        buf = (ctypes.c_uint8 * n)()
        self.lib.sim_flash_get(offset, buf, n)
        return bytes(buf)

    def load(self, image, offset=0):
        self.lib.sim_flash_load(offset, bytes(image), len(image))

    def ram(self, addr, n):
        buf = (ctypes.c_uint8 * n)()
        self.lib.sim_ram_get(addr, buf, n)
        return bytes(buf)

    def uart_send(self, port, data, baud=115200):
        self.lib.sim_uart_send(port, bytes(data), len(data), baud)

    def uart_recv(self, port, baud=115200):
        buf = (ctypes.c_uint8 * 8192)()
        n = self.lib.sim_uart_recv(port, buf, 8192, baud)
        return bytes(buf[:n])

    def usb_send(self, data):
        self.lib.sim_usb_send(bytes(data), len(data))

    def usb_recv(self):
        buf = (ctypes.c_uint8 * 65536)()
        n = self.lib.sim_usb_recv(buf, 65536)
        return bytes(buf[:n])

    def usb_descriptor(self, which):
        buf = (ctypes.c_uint8 * 256)()
        self.lib.sim_usb_descriptor(which, buf)
        return bytes(buf)

    def boot(self, keep_ram=False):
        self.lib.sim_power_on(1 if keep_ram else 0)
        return self.state()

    def write_handoff(self, flags=1, link=0, node_id=0, server=0, bitrate=0, baud=0, srxl2_id=0, path=b''):
        body = struct.pack('<HBBBBBBII', 20 + len(path), 1, flags, link, 0, node_id, server, bitrate, baud)
        body += struct.pack('<BBBB', srxl2_id, 0, 0, len(path)) + path
        crc = zlib_crc(body)
        self.lib.sim_ram_set(self.handoff_addr, struct.pack('<II', HANDOFF_TO_BL, crc) + body, 8 + len(body))

    def read_handoff(self):
        raw = self.ram(self.handoff_addr, 28)
        magic, crc, length, version, flags, link, attempts, node_id, server, bitrate, baud, srxl2 = \
            struct.unpack_from('<IIHBBBBBBIIB', raw)
        return {'magic': magic, 'crc_ok': crc == zlib_crc(self.ram(self.handoff_addr + 8, length)),
                'link': link, 'boot_attempts': attempts, 'node_id': node_id, 'bitrate': bitrate, 'baud': baud,
                'srxl2_id': srxl2}


def zlib_crc(data):
    import zlib
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF


def make_image(sim, size=20000, fmt='ap', board_id=None, seed=0, version=(1, 2), git_hash=0x1234abcd,
               desc_offset=0x200, sp=0x20004000, entry=None):
    """A plausible application: vector table, a descriptor after it, random code."""
    rng = random.Random(seed)
    img = bytearray(rng.getrandbits(8) for _ in range(size))
    struct.pack_into('<II', img, 0, sp, sim.app_base + 0x401 if entry is None else entry)
    if fmt == 'ap':
        img[desc_offset:desc_offset + mubl_image.AP_SIZE] = mubl_image.AP_SIG + bytes(mubl_image.AP_SIZE - 8)
        bid = sim.board_id if board_id is None else board_id
        return mubl_image.stamp(img, board_id=bid, version=version, git_hash=git_hash)
    if fmt == 'legacy':
        img[desc_offset:desc_offset + mubl_image.LEGACY_SIZE] = mubl_image.LEGACY_SIG + bytes(mubl_image.LEGACY_SIZE - 8)
        return mubl_image.stamp(img, version=version, git_hash=git_hash)
    # raw: make sure no signature appears by accident
    return bytes(img)
