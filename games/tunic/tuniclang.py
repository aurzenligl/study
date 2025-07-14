#!/usr/bin/env python

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List


# https://en.wikipedia.org/wiki/Letter_frequency
PAT = re.compile('[01]{6}:[01]{5}:[01]')
PAT3 = re.compile('[01]{6}:[01]{5}:[01]')
PAT2 = re.compile('[01]{6}:[01]{5}')
PAT1 = re.compile('[01]{6}')


@dataclass
class Char:
    innie: str
    outie: str
    dot: str
    tail: str


def cmd_count(path: Path):
    lines = path.read_text().splitlines()
    gramhist = {}
    chars = []
    for line in lines:
        parts = line.split()
        for part in parts:
            if char := parse_character(part):
                chars.append(char)
    for char in chars:
        innie, outie, dot = char
        v = f'i{innie}'
        if v != 'i000000':
            gramhist[v] = gramhist.get(v, 0) + 1
        v = f'o{outie}'
        if v != 'o00000':
            gramhist[v] = gramhist.get(v, 0) + 1
    total = sum(gramhist.values())
    for code, count in sorted(gramhist.items(), key=lambda k: -k[1]):
        print(f'{code} = {count/total*100:5.2f}%')


def cmd_substitute(path: Path, subs: List[str]):
    lines = path.read_text().splitlines()

    isubs = {'000000': ''}
    osubs = {'00000': ''}
    for s in subs:
        sidecode, letter = s.split(':')
        shortside, code = sidecode[0], sidecode[1:]
        if shortside == 'i':
            assert len(code) == 6
            isubs[code] = letter
        elif shortside == 'o':
            assert len(code) == 5
            osubs[code] = letter

    outlines = []
    for line in lines:
        outparts = []
        parts = line.split()
        for part in parts:

            if char := parse_character(part):
                innie, outie, dot = char
                i = isubs.get(innie, '_')
                o = osubs.get(outie, '_')
                d = "'" if dot == '1' else ''
                if dot == '1':
                    outparts.append(f'{o}{i}')
                else:
                    outparts.append(f'{i}{o}')
            else:
                outparts.append(part)
        outlines.append(' '.join(outparts))
    print(''.join(x + '\n' for x in outlines))


def parse_character(v: str):
    if PAT3.match(v):
        return v.split(':')
    if PAT2.match(v):
        return v.split(':') + ['0']
    if PAT1.match(v):
        return [v, '00000', '0']


def parse_opts():
    parser = argparse.ArgumentParser(prog='tuniclang-hacker')
    parser.add_argument('filename')
    parser.add_argument('-c', '--count', action='store_true')
    parser.add_argument('-s', '--substitute', metavar='TUNICCODE:LETTER', action='append')
    return parser.parse_args()


def main():
    opts = parse_opts()
    if opts.count:
        cmd_count(Path(opts.filename))
    elif opts.substitute:
        cmd_substitute(Path(opts.filename), opts.substitute)
    else:
        sys.exit('error: no operation chosen')


if __name__ == '__main__':
    main()


# o11000:a
