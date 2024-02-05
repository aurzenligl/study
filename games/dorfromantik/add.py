#!/usr/bin/env python

import sys
from util import Expr, symbol_by_letter, save_append

argv = sys.argv[1:]
assert len(argv) >= 2, 'expected at least label and patterns'
label = argv.pop(0)
assert all(p in symbol_by_letter for pattern in argv for p in pattern), 'invalid patterns'
patterns = [''.join(symbol_by_letter[c] for c in pat.ljust(6, 'q')) for pat in argv]
exprs = [Expr(pat, f'{label}-{chr(ord("a")+idx)}') for pat, idx in zip(patterns, range(len(argv)))]
print('-- adding to "slots" --')
for e in exprs:
    print(f'{e.pattern} {e.extra}')
save_append('slots', exprs)
