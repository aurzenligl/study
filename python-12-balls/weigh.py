#!/usr/bin/env python3

import itertools

def bit_and(bitsets):
    x = 2**24 - 1
    for bs in bitsets:
        x &= int(bs, 2)
    x = bin(x)[2:]
    return '0' * (len(bitsets[0]) - len(x)) + x

class Weigh:
    __slots__ = ['choice']

    def __init__(self, choice):
        self.choice = choice

    @property
    def table(self):
        x = self.choice
        lt = x.translate(''.maketrans('ab.', '100')) + x.translate(''.maketrans('ab.', '010'))
        gt = x.translate(''.maketrans('ab.', '010')) + x.translate(''.maketrans('ab.', '100'))
        eq = x.translate(''.maketrans('ab.', '001')) * 2
        return lt, gt, eq

    def analyze(self):
        table = self.table
        print('  ' + '0123456789ab' * 2)
        print('  ' + '-' * 12 + '+' * 12)
        print('<', table[0])
        print('>', table[1])
        print('=', table[2])

    def __repr__(self):
        return "Weigh('%s')" % self.choice

class Combination:
    __slots__ = ['weighs']

    def __init__(self, weighs):
        self.weighs = weighs

    @property
    def table(self):
        tables = [w.table for w in self.weighs]

        results = []
        for var in itertools.product(range(3), repeat=len(tables)):
            res = bit_and([tab[idx] for tab, idx in zip(tables, var)])
            results.append(res)
        return results

    @property
    def hist(self):
        hist = {}
        for res in self.table:
            found = res.count('1')
            hist[found] = hist.get(found, 0) + 1
        return hist

    @property
    def maxhist(self):
        return max(self.hist)

    def analyze(self):
        n = len(self.weighs)
        table = self.table
        print(' ' * (n + 1) + '0123456789ab' * 2)
        print(' ' * (n + 1) + '-' * 12 + '+' * 12)
        for sym, tab in zip(itertools.product('<>=', repeat=n), table):
            sym = ''.join(sym)
            print(sym, tab, tab.count('1'))

    def __repr__(self):
        return "Combination(%s)" % ', '.join(["'%s'" % w.choice for w in self.weighs])


x = Weigh('aaaabbbb....')
y = Weigh('bbbab...aaa.')

cb = Combination([x, y])

cb.analyze()


import pdb;pdb.set_trace()


# def perms_with_reps():
#   perms = set()
#   for p in itertools.product('ab.', repeat=12):
#     p = ''.join(p)
#     if not p.count('a') == 4 or not p.count('b') == 4:
#       continue
#     if p in perms:
#       continue
#     perms.add(p)
#   return list(perms)
# x = perms_with_reps()
# print(len(x))
# import pdb;pdb.set_trace()
