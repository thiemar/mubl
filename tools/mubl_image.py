#!/usr/bin/env python3
"""Application images for µBL: fill in descriptors, inspect them, make .apj files.

    mubl_image.py stamp app.bin [-o out.bin]          fill the descriptor's size and CRCs
    mubl_image.py info app.bin
    mubl_image.py apj app.bin --board-id N [-o app.apj]   for uploader.py / QGC / Mission Planner

Descriptors (the bootloader finds the first one after the first 8 bytes, word aligned):

  ArduPilot/PX4 (40 a2 e4 f1 64 68 91 06), 36 bytes:
      sig[8] crc1:u32 crc2:u32 image_size:u32 git_hash:u32 major:u8 minor:u8 board_id:u16 reserved[8]
      crc1 = CRC-32 of the image up to and including the signature,
      crc2 = CRC-32 from `major` to the end of the image (CRC-32: reflected, no inversion).
  Legacy "APDesc00" (thiemar, early PX4), 32 bytes:
      sig[8] image_crc:u64 image_size:u32 vcs_commit:u32 major:u8 minor:u8 reserved[6]
      image_crc = CRC-64-WE of the whole image with image_crc read as zero.

The application's build reserves the descriptor (filled with anything) and runs `stamp` on the
binary; `image_size` is the binary's length.
"""

import argparse
import base64
import json
import struct
import sys
import zlib

AP_SIG = bytes([0x40, 0xa2, 0xe4, 0xf1, 0x64, 0x68, 0x91, 0x06])
LEGACY_SIG = b'APDesc00'
AP_SIZE = 36
LEGACY_SIZE = 32
SCAN_START = 8


def crc32(data, crc=0):
    """ArduPilot's crc32_small: reflected 0xEDB88320, no initial or final inversion."""
    return (~zlib.crc32(bytes(data), ~crc & 0xFFFFFFFF)) & 0xFFFFFFFF


def _crc64_table():
    table = []
    for i in range(256):
        crc = i << 56
        for _ in range(8):
            crc = ((crc << 1) ^ 0x42F0E1EBA9EA3693 if crc & (1 << 63) else crc << 1) & 0xFFFFFFFFFFFFFFFF
        table.append(crc)
    return table


_CRC64 = _crc64_table()


def crc64_we(data):
    """CRC-64-WE: 0x42F0E1EBA9EA3693, MSB first, initial and final value all ones."""
    crc = 0xFFFFFFFFFFFFFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFFFFFFFFFFFFFF) ^ _CRC64[(crc >> 56) ^ b]
    return crc ^ 0xFFFFFFFFFFFFFFFF


def find_descriptor(image):
    for off in range(SCAN_START, len(image) - LEGACY_SIZE + 1, 4):
        sig = bytes(image[off:off + 8])
        if sig == AP_SIG:
            return 'ap', off
        if sig == LEGACY_SIG:
            return 'legacy', off
    return None, None


def stamp(image, board_id=None, version=None, git_hash=None):
    """Return a copy of `image` with its descriptor filled in."""
    image = bytearray(image)
    kind, off = find_descriptor(image)
    if kind is None:
        raise ValueError('no descriptor found')
    size = len(image)
    if kind == 'ap':
        if off + AP_SIZE > size:
            raise ValueError('descriptor runs past the end of the image')
        if board_id is not None:
            struct.pack_into('<H', image, off + 26, board_id)
        if version is not None:
            struct.pack_into('<BB', image, off + 24, *version)
        if git_hash is not None:
            struct.pack_into('<I', image, off + 20, git_hash)
        struct.pack_into('<I', image, off + 16, size)
        struct.pack_into('<II', image, off + 8, crc32(image[:off + 8]), crc32(image[off + 24:]))
    else:
        if version is not None:
            struct.pack_into('<BB', image, off + 24, *version)
        if git_hash is not None:
            struct.pack_into('<I', image, off + 20, git_hash)
        struct.pack_into('<I', image, off + 16, size)
        struct.pack_into('<Q', image, off + 8, 0)
        struct.pack_into('<Q', image, off + 8, crc64_we(image))
    return bytes(image)


def info(image):
    kind, off = find_descriptor(image)
    out = {'size': len(image), 'format': kind or 'none'}
    if len(image) >= 8:
        out['sp'], out['entry'] = struct.unpack_from('<II', image, 0)
    if kind == 'ap':
        crc1, crc2, size, git, major, minor, board = struct.unpack_from('<IIIIBBH', image, off + 8)
        ok = size == len(image) and crc1 == crc32(image[:off + 8]) and crc2 == crc32(image[off + 24:size])
        out.update(offset=off, image_size=size, git_hash=git, version='%d.%d' % (major, minor), board_id=board,
                   crc_ok=ok)
    elif kind == 'legacy':
        crc, size, vcs, major, minor = struct.unpack_from('<QIIBB', image, off + 8)
        blank = bytearray(image)
        struct.pack_into('<Q', blank, off + 8, 0)
        out.update(offset=off, image_size=size, vcs_commit=vcs, version='%d.%d' % (major, minor),
                   crc_ok=size == len(image) and crc == crc64_we(blank))
    return out


def apj(image, board_id, summary='mubl'):
    """The JSON firmware container uploader.py reads."""
    return json.dumps({
        'board_id': board_id,
        'magic': 'APJFWv1',
        'description': 'Firmware for a µBL board',
        'image': base64.b64encode(zlib.compress(bytes(image), 9)).decode('ascii'),
        'summary': summary,
        'version': '0.1',
        'image_size': len(image),
        'board_revision': 0,
        'git_identity': '',
    }, indent=4)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('stamp')
    s.add_argument('input')
    s.add_argument('-o', '--output')
    s.add_argument('--board-id', type=int)
    s.add_argument('--version', help='MAJOR.MINOR')
    s.add_argument('--git-hash', type=lambda v: int(v, 16))
    i = sub.add_parser('info')
    i.add_argument('input')
    j = sub.add_parser('apj')
    j.add_argument('input')
    j.add_argument('--board-id', type=int, required=True)
    j.add_argument('-o', '--output')
    args = ap.parse_args()
    data = open(args.input, 'rb').read()
    if args.cmd == 'stamp':
        version = tuple(int(x) for x in args.version.split('.')) if args.version else None
        out = stamp(data, args.board_id, version, args.git_hash)
        open(args.output or args.input, 'wb').write(out)
        print(json.dumps(info(out)))
    elif args.cmd == 'info':
        print(json.dumps(info(data), indent=2))
    else:
        open(args.output or args.input.rsplit('.', 1)[0] + '.apj', 'w').write(apj(data, args.board_id))


if __name__ == '__main__':
    sys.exit(main())
