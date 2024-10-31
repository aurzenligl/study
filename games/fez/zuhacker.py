#!/usr/bin/env python

import argparse
import sys
from pathlib import Path
from typing import List


def cmd_count(path: Path):
    total = 0
    zucount = {a+b: 0 for a in 'taevrb' for b in '0123'}
    lines = path.read_text().splitlines()
    for line in lines:
        if line.startswith('>'):
            line = line[2:]
            line = line.replace('  ', '')
            for code in split_zucodes(line):
                zucount[code] += 1
                total += 1
    for code, count in sorted(zucount.items(), key=lambda k: -k[1]):
        print(f'{code} = {count/total*100:5.2f}%')


def cmd_substitute(path: Path, subs: List[str]):
    zusubs = {a+b: '.' for a in 'taevrb' for b in '0123'}
    zusubs['  '] = ' '
    zusubs.update([s.split(':') for s in subs])
    lines = path.read_text().splitlines()
    for line in lines:
        if line.startswith('>'):
            for k, v in zusubs.items():
                line = line.replace(k, v)
            print(line)
        else:
            print(line)


def split_zucodes(text: str):
    return [text[(i*2):((i+1)*2)] for i in range(len(text)//2)]


def parse_opts():
    parser = argparse.ArgumentParser(prog='zulang-hacker')
    parser.add_argument('filename')
    parser.add_argument('-c', '--count', action='store_true')
    parser.add_argument('-s', '--substitute', metavar='ZUCODE:LETTER', action='append')
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
