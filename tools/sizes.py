#!/usr/bin/env python3
"""Flash, RAM and worst-case stack of every board, and of the F042 build with each link alone.

    tools/sizes.py [--markdown]

Flash is text + data of the linked image (LTO, -Os).  RAM is data + bss of the bootloader itself
(the reserved handoff block is outside it).  Stack is the deepest call chain from a GCC call-graph
build (-fcallgraph-info=su, no LTO, so frames are those of the non-inlined code: an upper bound),
plus the exception frame of the SysTick interrupt.
"""

import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VARIANTS = [
    ('f042-node', 'DroneCAN', '-DMUBL_DRONECAN=1 -DMUBL_SERIAL=0 -DMUBL_SRXL2=0'),
    ('f042-node', 'serial', '-DMUBL_DRONECAN=0 -DMUBL_SERIAL=1 -DMUBL_SRXL2=0'),
    ('f042-node', 'SRXL2', '-DMUBL_DRONECAN=0 -DMUBL_SERIAL=0 -DMUBL_SRXL2=1'),
    ('f042-node', 'serial + SRXL2', '-DMUBL_DRONECAN=0 -DMUBL_SERIAL=1 -DMUBL_SRXL2=1'),
]


def boards():
    return sorted(b for b in os.listdir(os.path.join(ROOT, 'boards')) if os.path.exists(os.path.join(ROOT, 'boards', b, 'board.mk')))


def make(board, build, extra='', callgraph=False):
    args = ['make', '--no-print-directory', 'BOARD=' + board, 'BUILD=' + build,
            'EXTRA_CFLAGS=-DMUBL_MEASURE ' + extra]
    if callgraph:
        args += ['LTO=', 'CALLGRAPH=1']
    subprocess.check_call(['rm', '-rf', os.path.join(ROOT, build)])
    subprocess.check_output(args, cwd=ROOT, stderr=subprocess.STDOUT)


def size(build):
    tool = os.path.join(ROOT, '..', 'toolchains', 'arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi',
                        'bin', 'arm-none-eabi-size')
    tool = tool if os.path.exists(tool) else 'arm-none-eabi-size'
    out = subprocess.check_output([tool, os.path.join(ROOT, build, 'mubl.elf')]).decode().splitlines()[1].split()
    text, data, bss = (int(x) for x in out[:3])
    return text + data, data + bss


def stack(build):
    """Deepest path through the call graph in the .ci (VCG) files."""
    nodes, edges = {}, {}
    for dirpath, _, files in os.walk(os.path.join(ROOT, build)):
        for f in files:
            if not f.endswith('.ci'):
                continue
            text = open(os.path.join(dirpath, f)).read()
            for m in re.finditer(r'node: \{ title: "([^"]+)" label: "([^"]*)"', text):
                title, label = m.groups()
                frame = re.search(r'(\d+) bytes \((static|dynamic|bounded)', label.replace('\\n', ' '))
                if frame:
                    nodes[title] = max(nodes.get(title, 0), int(frame.group(1)))
                else:
                    nodes.setdefault(title, 0)
            for m in re.finditer(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', text):
                edges.setdefault(m.group(1), set()).add(m.group(2))
    names = {}
    for t in nodes:
        names.setdefault(t.split(':')[-1], []).append(t)

    def resolve(target):
        if target in nodes:
            return [target]
        return names.get(target.split(':')[-1], [])

    # Calls through pointers: hal_guarded runs these (image.c, update.c).
    for caller in [n for n in nodes if n.split(':')[-1] == 'hal_guarded']:
        edges.setdefault(caller, set()).update(t for t in nodes if t.split(':')[-1] in ('check', 'blank_fn', 'area_crc'))
    memo = {}

    def depth(n, seen):
        if n in memo:
            return memo[n]
        if n in seen:
            return 0
        seen = seen | {n}
        best = 0
        for t in edges.get(n, ()):
            for r in resolve(t):
                best = max(best, depth(r, seen))
        memo[n] = nodes.get(n, 0) + best
        return memo[n]
    roots = [n for n in nodes if n.split(':')[-1] in ('Reset_Handler', 'main')]
    worst = max((depth(r, frozenset()) for r in roots), default=0)
    tick = max((depth(n, frozenset()) for n in nodes if n.split(':')[-1] == 'SysTick_Handler'), default=0)
    return worst + 32 + tick


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--markdown', action='store_true')
    args = ap.parse_args()
    rows = []
    for b in boards():
        make(b, 'build/sizes/' + b)
        flash, ram = size('build/sizes/' + b)
        make(b, 'build/sizes/' + b + '-cg', callgraph=True)
        rows.append((b, 'as configured', flash, ram, stack('build/sizes/' + b + '-cg')))
    for b, name, flags in VARIANTS:
        tag = re.sub(r'\W+', '-', name)
        make(b, 'build/sizes/%s-%s' % (b, tag), flags)
        flash, ram = size('build/sizes/%s-%s' % (b, tag))
        make(b, 'build/sizes/%s-%s-cg' % (b, tag), flags, callgraph=True)
        rows.append((b, name, flash, ram, stack('build/sizes/%s-%s-cg' % (b, tag))))
    print('| Board | Links | Flash | RAM | Stack (bound) |')
    print('| --- | --- | ---: | ---: | ---: |')
    for r in rows:
        print('| %s | %s | %d B | %d B | %d B |' % r)


if __name__ == '__main__':
    sys.exit(main())
