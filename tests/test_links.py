"""Link detection, priority and sharing."""

import struct
import unittest

from clients import OK, NoReply, SerialClient, SimSrxl2Transport
from sim import LINK_SERIAL, LINK_SRXL2, STATE_APP, STATE_BOOTLOADER, Sim, make_image
from test_srxl2 import handshake, srxl2

D, SER, SRX = 'MUBL_LINK_DRONECAN', 'MUBL_LINK_SERIAL', 'MUBL_LINK_SRXL2'


def priority(*links):
    return {'MUBL_LINK_PRIORITY': '"%s"' % ','.join(links)}


def config(*links, **extra):
    d = {'MUBL_LINK_PRIORITY': ','.join(links)}
    d.update(extra)
    return d


def can_frame(sim):
    """A NodeStatus from node 10, as any busy bus would carry."""
    data = bytes([0, 0, 0, 0, 0, 0, 0, 0xC0])
    return sim.lib.sim_rx((16 << 24) | (341 << 8) | 10, data, len(data), 0, 0)


class SharedPort(unittest.TestCase):
    def announcements(self, sim, ms=300):
        out = b''
        for _ in range(ms):
            sim.run(1)
            out += sim.uart_recv(0, 0)
        return [p for p in srxl2.Parser().feed(out) if p[1] == 0x21]

    def test_srxl2_first_announces_and_serial_still_claims(self):
        sim = Sim(1, config(D, SRX, SER)).fresh()
        sim.boot()
        self.assertEqual(len(self.announcements(sim)), 4)
        c = SerialClient(sim, 0)
        self.assertEqual(c.identify()['bl_rev'], 5)
        # the port is now the serial protocol's: SRXL2 no longer answers, even when addressed (the
        # serial protocol answers the stray bytes with INVALID, as ArduPilot's bootloader does)
        sim.uart_send(0, handshake(0x21, 0x40))
        sim.run(5)
        self.assertEqual(srxl2.Parser().feed(sim.uart_recv(0, 0)), [])

    def test_serial_first_keeps_srxl2_quiet(self):
        sim = Sim(1, config(D, SER, SRX)).fresh()
        sim.boot()
        self.assertEqual(self.announcements(sim), [])
        # a master that polls still finds the device: its first valid packet claims the port
        u = srxl2.Updater(SimSrxl2Transport(sim, 0), 0x40, log=lambda *a: None)
        self.assertEqual(u.handshake()['priority'], 1)
        with self.assertRaises(NoReply):
            SerialClient(sim, 0).command([0x21, 0x20], timeout_ms=50)

    def test_two_rates_take_turns(self):
        sim = Sim(1, config(D, SRX, SER, MUBL_SERIAL_BAUD=57600)).fresh()
        sim.boot()
        rates = []
        for _ in range(1000):
            sim.run(1)
            rates.append(sim.uart_baud(0))
        self.assertEqual(rates[0], 115200)                 # SRXL2 first: higher priority
        self.assertEqual(set(rates), {115200, 57600})
        switches = [i for i in range(1, len(rates)) if rates[i] != rates[i - 1]]
        self.assertEqual([b - a for a, b in zip(switches, switches[1:])], [250] * (len(switches) - 1))
        # a host at 57600 retrying GET_SYNC gets through within a slice, then the port stays put
        c = SerialClient(sim, 0, baud=57600)
        for _ in range(10):
            try:
                c.command([0x21, 0x20], timeout_ms=60)
                break
            except NoReply:
                pass
        else:
            self.fail('no sync at 57600')
        sim.run(1000)
        self.assertEqual(sim.uart_baud(0), 57600)
        self.assertEqual(c.identify()['bl_rev'], 5)


class Exclusive(unittest.TestCase):
    def test_detected_link_stops_lower_ones(self):
        sim = Sim(1, config(D, SRX, SER, MUBL_LINK_EXCLUSIVE=1)).fresh()
        sim.boot()
        sim.run(10)
        self.assertEqual(can_frame(sim), 1)
        sim.run(5)
        self.assertEqual(sim.uart_baud(0), 0)              # SRXL2 and serial stopped
        self.assertNotEqual(sim.can_bitrate(), 0)

    def test_lower_link_detected_first_keeps_higher_ones(self):
        sim = Sim(1, config(D, SER, SRX, MUBL_LINK_EXCLUSIVE=1, MUBL_SRXL2_PORT=1)).fresh()
        sim.boot()
        SerialClient(sim, 0).sync()                        # serial detected
        sim.run(5)
        self.assertEqual(sim.uart_baud(1), 0)              # SRXL2 (lower) stopped
        self.assertNotEqual(sim.can_bitrate(), 0)          # DroneCAN (higher) still listening
        self.assertEqual(can_frame(sim), 1)

    def test_shared_mode_keeps_everything(self):
        sim = Sim(1, config(D, SRX, SER)).fresh()
        sim.boot()
        sim.run(10)
        can_frame(sim)
        sim.run(5)
        self.assertEqual(sim.uart_baud(0), 115200)


class Policy(unittest.TestCase):
    def test_handoff_names_a_link(self):
        sim = Sim(1).fresh()
        sim.load(make_image(sim))
        sim.write_handoff(flags=1, link=LINK_SERIAL, baud=230400)
        sim.boot(keep_ram=True)
        self.assertEqual(sim.can_bitrate(), 0)             # DroneCAN not started
        self.assertEqual(sim.uart_baud(0), 230400)          # serial alone, at the rate it was given
        self.assertEqual(SerialClient(sim, 0, baud=230400).identify()['board_id'], sim.board_id)

    def test_handoff_srxl2_device_id(self):
        sim = Sim(1).fresh()
        sim.write_handoff(flags=1, link=LINK_SRXL2, srxl2_id=0x50)
        sim.boot(keep_ram=True)
        u = srxl2.Updater(SimSrxl2Transport(sim, 0), 0x50, log=lambda *a: None)
        u.handshake()
        self.assertEqual(u.info()['board_id'], sim.board_id)

    def test_detection_extends_boot_delay(self):
        sim = Sim(1, {'MUBL_DETECT_EXTEND_MS': 2000}).fresh()
        sim.load(make_image(sim))
        sim.boot()
        sim.run(100)
        can_frame(sim)
        self.assertEqual(sim.run(2500), STATE_BOOTLOADER)
        self.assertEqual(sim.run(600), STATE_APP)

    def test_traffic_alone_does_not_hold(self):
        """A busy bus (CAN traffic, an SRXL2 master) must not keep a valid application from starting."""
        sim = Sim(1).fresh()
        sim.load(make_image(sim))
        sim.boot()
        ctrl = srxl2.packet(0xCD, bytes(9))
        for _ in range(120):
            can_frame(sim)
            sim.uart_send(0, ctrl)
            if sim.run(11) == STATE_APP:
                break
        self.assertEqual(sim.state(), STATE_APP)
        self.assertLess(sim.now(), 1100)


if __name__ == '__main__':
    unittest.main()
