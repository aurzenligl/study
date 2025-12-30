#!/usr/bin/env python

import sys

def sub(eq, i, c):
    return eq[:i] + c + eq[i + 1:]

def mov(eq):
    trans = {
        '2': ['3'],
        '3': ['2'],
    }
    for i in range(len(eq)):
        for r in trans.get(eq[i], []):
            yield sub(eq, i, r)

def rem(eq):
    trans = {
        '6': ['5'],
        '7': ['1'],
        '8': ['0', '6', '9'],
        '9': ['3', '5'],
        '+': ['-'],
    }
    for i in range(len(eq)):
        for r in trans.get(eq[i], []):
            yield sub(eq, i, r)

def add(eq):
    trans = {
        '0': ['8'],
        '1': ['7'],
        '3': ['9'],
        '5': ['6', '9'],
        '6': ['8'],
        '9': ['8'],
        '-': ['+'],
    }
    for i in range(len(eq)):
        for r in trans.get(eq[i], []):
            yield sub(eq, i, r)

def mutate(eq, mutators, consume):
    if not mutators:
        return consume(eq)
    for changed in mutators[0](eq):
        mutate(changed, mutators[1:], consume)

def eval_or_none(eq):
    try:
        return eval(eq.replace('=', '=='))
    except SyntaxError as e:
        return None

def main(eq):
    def solve(eq):
        if eval_or_none(eq) is True:
            print('solution:', eq)

    mutate(eq, [mov], solve)
    mutate(eq, [rem, add], solve)

if __name__ == '__main__':
    assert len(sys.argv) == 2
    main(sys.argv[1])
