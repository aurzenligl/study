#!/usr/bin/env python3

from weigh import Weigh, Combination

winhist = 24
winners = None
for z in Weigh.all_444():
    x = Weigh('aaaabbbb....')
    y = Weigh('...aaaab.bbb')
    cb = Combination([x, y, z])
    maxhist = cb.maxhist
    if maxhist == winhist:
        winners.append(cb)
    if maxhist < winhist:
        winhist = maxhist
        winners = [cb]

unique_combs = {tuple(winners[0].hist.items()) : w for w in winners}
winner = winners[0]

print('minimum bits:', winhist)
print('solutions:', len(winners))
print('unique solutions (wrt hist):', len(unique_combs))
print('first solution:', winner)
print()
winner.analyze()
