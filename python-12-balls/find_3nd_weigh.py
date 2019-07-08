#!/usr/bin/env python3

from weigh import Weigh, Combination

weighs = Weigh.all_444()

minhist = 24
winner = None
for z in weighs:
    x = Weigh('aaaabbbb....')
    y = Weigh('...aaaab.bbb')
    cb = Combination([x, y, z])
    if cb.maxhist < minhist:
        minhist = cb.maxhist
        winner = cb
print('winner:', winner)
print('minhist:', minhist)
print()
winner.analyze()
