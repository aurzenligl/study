#!/usr/bin/env python

import argparse
import re
import sys
from colorama import Fore
from dataclasses import dataclass
from pathlib import Path
from typing import List


BUILTIN_OUTIES = {
    '11100': 'æ',   # and, grant, shadow, cast, that
    '01100': 'ɔː',  # sought, gnaw, all
    '00011': 'ɪ',   # in, is, live, ruin
    '00111': 'e',   # open, held, self, thread
    '00110': 'ʊ',   # look, understood, faithful
    '11000': 'ə',   # but, one, must

    '01111': 'i',   # between, holy, seek, be
    '11110': 'u',   # to, you, truth, tomb
    '10111': 'əʳ',  # power, nature, manner, seeker, were
    '11101': 'ɔːʳ', # shore, your, or, for, glory
    '11011': 'ɑːʳ', # far, apart, arc, sarcophagus
    '01101': 'ɪəʳ', # here, disappear, hero

    '01000': 'eɪ',  # wait, same, break, plain, great
    '10000': 'aɪ',  # lie, outside, arrive, life
    '00010': 'ɔɪ',  # coil, spoil, turmoil, boil
    '00001': 'aʊ',  # about, outside, power
    '11111': 'oʊ',  # no, open, hole, old (UK: əʊ)
    '00101': 'eəʳ', # their, heir, careful, bear
}

BUILTIN_INNIES = {
    '000101': 'm',  # from, become, wisdom
    '001101': 'n',  # and, plain, one
    '111111': 'ŋ',  # ring, seeking, alluring
    '100010': 'p',  # plain, palace, open
    '010001': 'b',  # build, become, but
    '101010': 't',  # to, tomb, sought, built

    '010101': 'd',  # do, discover, and, lived
    '110001': 'k',  # cast, secret, sacred, seek
    '100011': 'g',  # grow, glory, begin
    '010100': 'dʒ', # origin, gene, jam
    '001010': 'tʃ', # which, nature, venture, future
    '100110': 'f',  # for, fable, life, self

    '011001': 'v',  # give, leave, of, cover
    '111010': 'θ',  # thread, thunder, truth, with
    '010111': 'ð',  # the, they, there, those, this
    '110110': 's',  # so, cycle, seeker, quest, curse
    '011011': 'z',  # those, lives, as, is, was
    '101111': 'ʃ',  # shore, shatter, sure, constradiction, potential

    '111101': 'ʒ',  # usual, treasure, vision
    '010011': 'h',  # have, holy, held, here, perhaps
    '110010': 'r',  # real, reveal, ruin, true
    '011010': 'j',  # you, new, future, continue
    '101000': 'w',  # will, wait, were, quest
    '010010': 'l',  # long, life, will, hole
}


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


def cmd_substitute(path: Path, subs: List[str], unders: List[str], raw: bool):
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

    iunders = set()
    ounders = set()
    for u in unders:
        shortside, code = u[0], u[1:]
        if shortside == 'i':
            assert len(code) == 6
            iunders.add(code)
        elif shortside == 'o':
            assert len(code) == 5
            ounders.add(code)

    outlines = []
    for line in lines:
        last_part_was_a_char = False
        outparts = []
        rawparts = []
        parts = line.split()
        for part in parts:
            if char := parse_character(part):
                innie, outie, dot = char

                i0, i1 = Fore.GREEN, Fore.RESET
                if innie in iunders:
                    i0, i1 = Fore.RED, Fore.RESET

                o0, o1 = Fore.LIGHTGREEN_EX, Fore.RESET
                if outie in ounders:
                    o0, o1 = Fore.LIGHTRED_EX, Fore.RESET

                i = i0 + isubs.get(innie, '_') + i1
                o = o0 + osubs.get(outie, '_') + o1

                if dot == '1':
                    if last_part_was_a_char:
                        outparts[-1] = outparts[-1] + f'{o}{i}'
                    else:
                        outparts.append(f'{o}{i}')
                    rawparts.append(f"{outie if '1' in outie else ''}:{innie if '1' in innie else ''}".strip(':'))
                else:
                    if last_part_was_a_char:
                        outparts[-1] = outparts[-1] + f'{i}{o}'
                    else:
                        outparts.append(f'{i}{o}')
                    rawparts.append(f"{innie if '1' in innie else ''}:{outie if '1' in outie else ''}".strip(':'))
                last_part_was_a_char = True
            else:
                outparts.append(part)
                rawparts.append(part)
                last_part_was_a_char = False
        outlines.append(' '.join(outparts))
        if raw:
            outlines.append(' '.join(rawparts))
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
    parser.add_argument('-s', '--substitute', metavar='TUNICCODE:LETTER', action='append', default=[])
    parser.add_argument('-u', '--underscore', metavar='TUNICCODE', action='append', default=[])
    parser.add_argument('-r', '--raw', action='store_true')
    return parser.parse_args()


def main():
    opts = parse_opts()
    ibltins = [f'i{code}:{letter}' for code, letter in BUILTIN_INNIES.items()]
    obltins = [f'o{code}:{letter}' for code, letter in BUILTIN_OUTIES.items()]
    opts.substitute = ibltins + obltins + opts.substitute
    if opts.count:
        cmd_count(Path(opts.filename))
    else:
        cmd_substitute(Path(opts.filename), opts.substitute, opts.underscore or [], opts.raw)


if __name__ == '__main__':
    main()


# o11000:a
