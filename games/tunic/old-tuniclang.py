#!/usr/bin/env python

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List


# https://en.wikipedia.org/wiki/Letter_frequency
PAT = re.compile('[01]+')


@dataclass
class Bar:
    top: str
    bottom: str


def cmd_count(path: Path):
    lines = path.read_text().splitlines()
    bars: List[Bar] = []
    while lines:
        line = lines.pop(0)
        if not line.strip():
            continue
        top, bottom = line.strip(), lines.pop(0).strip()
        assert PAT.sub('?', top) == PAT.sub('?', bottom), f'{top} != {bottom}'
        bars.append(Bar(top, bottom))
    gramhist = {}
    def check_one_side(side):
        grams = []
        for b in bars:
            grams += PAT.findall(getattr(b, side))
        for g in grams:
            assert len(g) == 7
        for g in grams:
            v = side[0] + g
            gramhist[v] = gramhist.get(v, 0) + 1
    check_one_side('top')
    check_one_side('bottom')
    total = sum(gramhist.values())
    for code, count in sorted(gramhist.items(), key=lambda k: -k[1]):
        print(f'{code} = {count/total*100:5.2f}%')


def cmd_substitute(path: Path, subs: List[str]):
    lines = path.read_text().splitlines()
    bars: List[Bar] = []
    while lines:
        line = lines.pop(0)
        if not line.strip():
            continue
        top, bottom = line.strip(), lines.pop(0).strip()
        assert PAT.sub('?', top) == PAT.sub('?', bottom), f'{top} != {bottom}'
        bars.append(Bar(top, bottom))

    for s in subs:
        sidecode, letter = s.split(':')
        shortside, code = sidecode[0], sidecode[1:]
        assert len(code) == 7
        side = {'t': 'top', 'b': 'bottom'}[shortside]

        for b in bars:
            v = getattr(b, side)
            x = re.sub(code, letter, v)
            setattr(b, side, x)

    for b in bars:
        b.top = re.sub('0000000', ' ', b.top)
        b.bottom = re.sub('0000000', ' ', b.bottom)

    for b in bars:
        b.top = PAT.sub('_', b.top)
        b.bottom = PAT.sub('_', b.bottom)

    for b in bars:
        print(b.top)
        print(b.bottom)
        print()


def cmd_migrate(path: Path):
    newlines = []
    lines = path.read_text().splitlines()
    while lines:
        line = lines.pop(0)
        if not line.strip():
            continue
        top, bottom = line.strip(), lines.pop(0).strip()
        assert PAT.sub('?', top) == PAT.sub('?', bottom), f'{top} != {bottom}'
        b = Bar(top, bottom)

        newb = b.top
        for a, b in zip(PAT.findall(b.top), PAT.findall(b.bottom)):
            assert len(a) == 7
            assert len(b) == 7
            assert a[0] == b[0]
            innie = a[3] + a[5] + a[2] + b[1] + b[5] + b[4]
            outie = a[4] + a[1] + a[0] + b[2] + b[3]
            dot = b[6]
            char = f'{innie}:{outie}:{dot}'
            assert len(char) == 14
            newb = newb.replace(a, char, 1)
        newlines.append(newb)
    print(''.join(x + '\n' for x in newlines))


def parse_opts():
    parser = argparse.ArgumentParser(prog='tuniclang-hacker')
    parser.add_argument('filename')
    parser.add_argument('-c', '--count', action='store_true')
    parser.add_argument('-s', '--substitute', metavar='TUNICCODE:LETTER', action='append')
    parser.add_argument('-m', '--migrate', action='store_true')
    return parser.parse_args()


def main():
    opts = parse_opts()
    if opts.count:
        cmd_count(Path(opts.filename))
    elif opts.substitute:
        cmd_substitute(Path(opts.filename), opts.substitute)
    elif opts.migrate:
        cmd_migrate(Path(opts.filename))
    else:
        sys.exit('error: no operation chosen')


if __name__ == '__main__':
    main()


# ./tuniclang.py tuniclang.txt -s b0000010:e -s t0000011:e -s b0100100:a -s t0100100:a -s t1100100:o -s b1001000:r -s b0100010:t

# frequency hint
# -s t0000011:e -s b0000010:e

# "a" hint
# -s t0100100:a -s b0100100:a

# "hero or a fool" hint:
# -s t1100100:o -s b1001000:r
