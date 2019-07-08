#!/usr/bin/env python3

from weigh import Weigh, Combination

weighs = Weigh.all_444()

minhist = 24
winner = None
for y in weighs:
    x = Weigh('aaaabbbb....')
    cb = Combination([x, y])
    if cb.maxhist < minhist:
        minhist = cb.maxhist
        winner = cb
print('winner:', winner)
print('minhist:', minhist)
print()
winner.analyze()
