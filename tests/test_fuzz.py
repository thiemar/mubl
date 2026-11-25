"""tests/fuzz.c: random and well-formed-but-random traffic on every link, with AddressSanitizer and
UBSan where the compiler has them, with USB on the F0/F3 peripheral model and (H7) the OTG model.  MUBL_FUZZ_ITERATIONS sets the length (default 200 per geometry)."""

import os
import subprocess
import unittest

from sim import BUILD, CC, ROOT, SOURCES


def build(geometry, sanitize, otg=0):
    out = os.path.join(BUILD, 'fuzz_%d%s%s' % (geometry, '_otg' if otg else '', '_san' if sanitize else ''))
    srcs = [os.path.join(ROOT, p) for p in SOURCES + ['src/dronecan/classic/dronecan.c', 'tests/fuzz.c']]
    if os.path.exists(out) and os.path.getmtime(out) >= max(os.path.getmtime(p) for p in srcs):
        return out
    os.makedirs(BUILD, exist_ok=True)
    san = ['-fsanitize=address,undefined', '-fno-sanitize-recover=undefined', '-fno-omit-frame-pointer'] if sanitize else []
    subprocess.check_call([CC, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror'] + san +
                          ['-DSIM_GEOMETRY=%d' % geometry, '-DMUBL_USB_OTG_CORE=%d' % otg,
                           '-I' + os.path.join(ROOT, 'boards/sim'),
                           '-I' + os.path.join(ROOT, 'src'), '-I' + os.path.join(ROOT, 'src/dronecan/classic')] +
                          srcs + ['-o', out], stderr=subprocess.DEVNULL if sanitize else None)
    return out


class Fuzz(unittest.TestCase):
    def run_geometry(self, geometry, otg=0):
        try:
            exe = build(geometry, True, otg)
        except subprocess.CalledProcessError:
            exe = build(geometry, False, otg)       # no sanitizers on this compiler
        n = os.environ.get('MUBL_FUZZ_ITERATIONS', '200')
        r = subprocess.run([exe, n, str(1000 + geometry)], capture_output=True, text=True, timeout=1800)
        self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-4000:])

    def test_f0(self):
        self.run_geometry(0)

    def test_f4(self):
        self.run_geometry(1)

    def test_h7(self):
        self.run_geometry(2)

    def test_h7_otg(self):
        """USB on the OTG core model (the others run the F0/F3 USB peripheral model)."""
        self.run_geometry(2, otg=1)


if __name__ == '__main__':
    unittest.main()
