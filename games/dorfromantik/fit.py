#!/usr/bin/env python

import sys
from util import Expr, ExprError, symbol_by_letter, load, glob_index

assert len(sys.argv) == 2, 'expected one arg, a pattern'
arg = sys.argv[1]
assert len(arg) == 6, 'expected 6-letter pattern'
pattern = ''.join([symbol_by_letter[c] for c in arg])
print(pattern)

fit6 = []
fit5 = []
exprs = load('slots')
for e in exprs:
    var6 = [pattern[i:] + pattern[:i] for i in range(6)]
    var5 = [glob_index(pattern[i:] + pattern[:i], j) for i in range(6) for j in range(6)]
    if any(e.match(v) for v in var6):
        fit6.append(e)
    elif any(e.match(v) for v in var5):
        fit5.append(e)

if fit6:
    print('--')
    for f in fit6:
        print(f'[6] {f.pattern} {f.extra}')
if fit5:
    print('--')
    for f in fit5:
        print(f'[5] {f.pattern} {f.extra}')
if not fit6 + fit5:
    print('-- no matches --')
