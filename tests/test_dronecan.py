"""DroneCAN link against sfwa-dronecan's Python stack (the pydronecan API) on its virtual bus.

The bootloader runs as a C node (vbus CNodeLibrary); the other side uses the stock application
classes: CentralizedServer for allocation, NodeMonitor, FileServer and FirmwareUpdater.
Needs the sfwa-dronecan checkout next to this repository (or SFWA_DRONECAN=path).
"""

import os
import struct
import sys
import tempfile
import unittest

from clients import OK, SerialClient
from sim import (APP_BAD_BOARD, APP_OK, LINK_DRONECAN, ROOT, STATE_APP, STATE_BOOTLOADER, Sim, make_image,
                 mubl_image)

DC = os.environ.get('SFWA_DRONECAN', os.path.join(ROOT, '..', 'dronecan'))
sys.path.insert(0, os.path.join(DC, 'python'))
try:
    import dronecan
    from dronecan.node import Node
    from sfwa_dronecan.vbus import CNodeLibrary, VirtualBus, VirtualClock
except Exception as e:      # noqa: BLE001
    dronecan = None
    SKIP = 'sfwa-dronecan not available: %s' % e

MODE_MAINTENANCE, MODE_SOFTWARE_UPDATE = 2, 3
HEALTH_ERROR = 2


@unittest.skipIf(dronecan is None, 'sfwa-dronecan not available')
class DroneCAN(unittest.TestCase):
    geometry = 1

    def setUp(self):
        self.sim = Sim(self.geometry).fresh()
        self.clock = VirtualClock()
        self.bus = VirtualBus(self.clock)
        self.sim.set_time(int(self.clock() * 1000))
        self.c = CNodeLibrary(self.bus, self.sim.path, 'mubl')
        self.tmp = tempfile.mkdtemp()
        self.closers = []

    def tearDown(self):
        for c in self.closers:
            c.close()

    def node(self, node_id=None, **kw):
        return Node(self.bus.attach(), node_id=node_id, clock=self.clock, **kw)

    def server(self, image=None, allocate=True):
        """Node 127: allocator, monitor and file server for app.bin."""
        s = self.node(127)
        self.monitor = dronecan.app.node_monitor.NodeMonitor(s)
        self.closers.append(self.monitor)
        if allocate:
            alloc = dronecan.app.dynamic_node_id.CentralizedServer(s, self.monitor)
            self.closers.append(alloc)
        if image is not None:
            with open(os.path.join(self.tmp, 'app.bin'), 'wb') as f:
                f.write(image)
        self.files = dronecan.app.file_server.FileServer(s, lookup_paths=[self.tmp])
        self.closers.append(self.files)
        self.logs = []
        s.add_handler(dronecan.uavcan.protocol.debug.LogMessage, lambda e: self.logs.append(e.message.text.decode()))
        self.status = {}
        s.add_handler(dronecan.uavcan.protocol.NodeStatus,
                      lambda e: self.status.__setitem__(e.transfer.source_node_id, e.message))
        return s

    def run_until(self, cond, seconds, nodes):
        for _ in range(int(seconds * 100)):
            self.bus.run(0.01, nodes=nodes)
            if cond():
                return True
        return False

    def test_allocation_and_node_info(self):
        img = make_image(self.sim, version=(3, 7), git_hash=0xdeadbeef)
        self.sim.load(img)
        self.sim.set_hold(1)
        self.sim.boot()
        s = self.server()
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        nid = self.sim.node_id()
        self.assertTrue(self.run_until(lambda: self.monitor.exists(nid) and self.monitor.get(nid).info, 5, [s]))
        info = self.monitor.get(nid).info
        self.assertEqual(info.name.decode(), 'org.sfwa.mubl.sim')
        self.assertEqual((info.software_version.major, info.software_version.minor), (3, 7))
        self.assertEqual(info.software_version.vcs_commit, 0xdeadbeef)
        crc1, crc2 = struct.unpack_from('<II', img, 0x200 + 8)
        self.assertEqual(info.software_version.image_crc, crc2 << 32 | crc1)
        self.assertEqual(bytes(info.hardware_version.unique_id)[:12], bytes(range(0xA0, 0xAC)))
        self.assertEqual(info.hardware_version.major, 9999 >> 8)
        self.assertEqual(self.status[nid].mode, MODE_MAINTENANCE)
        self.assertEqual(self.sim.errors(), 0)

    def test_autobaud(self):
        self.sim.can_bus_bitrate(250000)
        self.sim.boot()
        s = self.server()
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 15, [s]))
        self.assertEqual(self.sim.can_bitrate(), 250000)
        self.assertEqual(self.sim.can_listen_only(), 0)
        self.assertEqual(self.sim.errors(), 0)          # nothing was sent while listening

    def test_silent_bus_cycles(self):
        self.sim.boot()
        rates = set()
        for _ in range(60):
            self.bus.run(0.1)
            rates.add(self.sim.can_bitrate())
            self.assertEqual(self.sim.can_listen_only(), 1)
        self.assertEqual(rates, {1000000, 500000, 250000, 125000})

    def update_via_updater(self, image):
        s = self.server(image)
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        nid = self.sim.node_id()
        done = []
        dronecan.app.firmware_update.FirmwareUpdater(s, nid, os.path.join(self.tmp, 'app.bin'),
                                                    on_done=lambda ok, why: done.append((ok, why)))
        self.assertTrue(self.run_until(lambda: done, 120, [s]))
        return done[0], nid

    def test_firmware_update(self):
        self.sim.app_emulation(1)
        img = make_image(self.sim, size=70001, seed=5)
        self.sim.boot()
        (ok, why), nid = self.update_via_updater(img)
        self.assertTrue(ok, why)
        self.assertEqual(self.sim.state(), STATE_APP)
        self.assertEqual(self.sim.flash(0, len(img)), img)
        self.assertEqual(self.sim.read_handoff()['node_id'], nid)
        self.assertIn('update complete', self.logs)
        self.assertEqual(self.sim.errors(), 0)

    def test_rejected_image(self):
        img = make_image(self.sim, board_id=1)
        self.sim.boot()
        s = self.server(img)
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        nid = self.sim.node_id()
        req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
            source_node_id=127, image_file_remote_path=dronecan.uavcan.protocol.file.Path(path='app.bin'))
        got = []
        s.request(req, nid, got.append)
        self.assertTrue(self.run_until(lambda: 'image rejected' in self.logs, 30, [s]))
        self.assertEqual(got[0].response.error, 0)
        self.run_until(lambda: False, 1.5, [s])
        self.assertEqual(self.status[nid].health, HEALTH_ERROR)
        self.assertEqual(self.sim.app_status(), APP_BAD_BOARD)
        self.assertEqual(self.sim.state(), STATE_BOOTLOADER)

    def test_update_requested_by_ap_periph(self):
        """AP_Periph's app_bootloader_comms: node ID, server and path; the bootloader reads at once."""
        img = make_image(self.sim, size=30000, seed=6)
        self.sim.load(make_image(self.sim, seed=7))
        comms = struct.pack('<IIIII', 0xC544AD9A, 0, 0, 0, 0) + bytes([127, 42]) + b'app.bin\0'
        self.sim.lib.sim_ram_set(self.sim.lib.sim_ap_comms_addr(), comms, len(comms))
        self.sim.boot(keep_ram=True)
        s = self.server(img, allocate=False)
        self.assertTrue(self.run_until(lambda: self.sim.state() == STATE_APP, 30, [s]))
        self.assertEqual(self.sim.node_id(), 42)
        self.assertEqual(self.sim.flash(0, len(img)), img)

    def test_update_requested_by_handoff(self):
        img = make_image(self.sim, size=30000, seed=8)
        self.sim.load(make_image(self.sim, seed=9))
        self.sim.can_bus_bitrate(500000)
        self.sim.write_handoff(flags=1, link=LINK_DRONECAN, node_id=50, server=127, bitrate=500000, path=b'app.bin')
        self.sim.boot(keep_ram=True)
        self.assertEqual(self.sim.can_bitrate(), 500000)       # no autobaud
        self.assertEqual(self.sim.can_listen_only(), 0)
        s = self.server(img, allocate=False)
        self.assertTrue(self.run_until(lambda: self.sim.state() == STATE_APP, 30, [s]))
        self.assertEqual(self.sim.flash(0, len(img)), img)
        h = self.sim.read_handoff()
        self.assertEqual((h['node_id'], h['bitrate'], h['link']), (50, 500000, LINK_DRONECAN))

    def test_px4_handoff(self):
        """PX4/thiemar shared structure in the bxCAN filter registers, both directions."""
        import ctypes
        self.sim.app_emulation(1)
        self.sim.load(make_image(self.sim, seed=1))
        sig, rate, nid = 0xB0A04150, 1000000, 33
        crc = mubl_image.crc64_we(struct.pack('<III', sig, rate, nid))
        self.sim.lib.sim_px4_set((ctypes.c_uint32 * 5)(crc >> 32, crc & 0xFFFFFFFF, sig, rate, nid))
        self.sim.boot()
        self.assertEqual(self.sim.can_bitrate(), rate)               # no autobaud
        self.assertEqual(self.sim.node_id(), nid)                    # no allocation
        self.assertEqual(self.sim.px4_get(2), 0)                     # consumed
        img = make_image(self.sim, size=20000, seed=2)
        s = self.server(img, allocate=False)
        self.assertFalse(self.run_until(lambda: self.sim.state() != STATE_BOOTLOADER, 3, [s]))   # the app asked to stay
        req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
            source_node_id=127, image_file_remote_path=dronecan.uavcan.protocol.file.Path(path='app.bin'))
        s.request(req, nid, lambda e: None)
        self.assertTrue(self.run_until(lambda: self.sim.state() == STATE_APP, 30, [s]))
        w = [self.sim.px4_get(i) for i in range(5)]
        self.assertEqual(w[2:], [0xB0A0424C, rate, nid])            # the bootloader's signature for the app
        self.assertEqual(w[0] << 32 | w[1], mubl_image.crc64_we(struct.pack('<III', *w[2:])))

    def test_restart_node(self):
        self.sim.boot()
        s = self.server()
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        boots = self.sim.boots()
        got = []
        s.request(dronecan.uavcan.protocol.RestartNode.Request(magic_number=0xACCE551B1E), self.sim.node_id(),
                  got.append)
        self.assertTrue(self.run_until(lambda: self.sim.boots() > boots, 2, [s]))
        self.assertTrue(got and got[0].response.ok)

    def test_busy_while_serial_updates(self):
        self.sim.boot()
        s = self.server(make_image(self.sim))
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        c = SerialClient(self.sim, port=0)
        c.identify()
        self.assertEqual(c.erase(), OK)
        req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
            source_node_id=127, image_file_remote_path=dronecan.uavcan.protocol.file.Path(path='app.bin'))
        got = []
        s.request(req, self.sim.node_id(), got.append)
        self.assertTrue(self.run_until(lambda: got, 2, [s]))
        self.assertEqual(got[0].response.error, 1)       # ERROR_INVALID_MODE: another link owns the update

    def test_server_lost_then_retried(self):
        self.sim.app_emulation(1)
        img = make_image(self.sim, size=60000, seed=12)
        self.sim.boot()
        s = self.server(img)
        self.assertTrue(self.run_until(lambda: self.sim.node_id() != 0, 10, [s]))
        nid = self.sim.node_id()
        req = dronecan.uavcan.protocol.file.BeginFirmwareUpdate.Request(
            source_node_id=127, image_file_remote_path=dronecan.uavcan.protocol.file.Path(path='app.bin'))
        s.request(req, nid, lambda e: None)
        self.assertTrue(self.run_until(lambda: self.status.get(nid) and self.status[nid].mode == MODE_SOFTWARE_UPDATE,
                                       5, [s]))
        self.files.close()                                # the file server goes away mid-update
        self.closers.remove(self.files)
        self.assertTrue(self.run_until(lambda: 'file server not responding' in self.logs, 20, [s]))
        self.files = dronecan.app.file_server.FileServer(s, lookup_paths=[self.tmp])
        self.closers.append(self.files)
        s.request(req, nid, lambda e: None)
        self.assertTrue(self.run_until(lambda: self.sim.state() == STATE_APP, 60, [s]))
        self.assertEqual(self.sim.flash(0, len(img)), img)
        self.assertEqual(self.sim.app_status(), APP_OK)


class DroneCANH7(DroneCAN):
    geometry = 2


if __name__ == '__main__':
    unittest.main()
