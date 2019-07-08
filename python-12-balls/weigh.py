#!/usr/bin/env python3

import itertools

def bitstring(bits, n):
    text = bin(bits)[2:]
    return '0' * (n - len(text)) + text

def bitand(bitsets):
    x = bitsets[0]
    for bs in bitsets[1:]:
        x &= bs
    return x

class Weigh:
    __slots__ = ['choice']

    lt_tr = ''.maketrans('ab.', '100')
    gt_tr = ''.maketrans('ab.', '010')
    eq_tr = ''.maketrans('ab.', '001')

    def __init__(self, choice):
        self.choice = choice

    @staticmethod
    def all_444():
        perms = set()
        for p in itertools.product('ab.', repeat=12):
            p = ''.join(p)
            if p.count('a') != 4 or p.count('b') != 4:
                continue
            if p in perms:
                continue
            perms.add(p)
        return [Weigh(p) for p in sorted(perms)]

    @property
    def table(self):
        lt_bits = int(self.choice.translate(self.lt_tr), 2)
        gt_bits = int(self.choice.translate(self.gt_tr), 2)
        eq_bits = int(self.choice.translate(self.eq_tr), 2)

        lt = (lt_bits << 12) | gt_bits
        gt = (gt_bits << 12) | lt_bits
        eq = (eq_bits << 12) | eq_bits

        return lt, gt, eq

    def analyze(self):
        table = self.table
        print('  ' + '0123456789ab' * 2)
        print('  ' + '-' * 12 + '+' * 12)
        print('<', bitstring(table[0], 24))
        print('>', bitstring(table[1], 24))
        print('=', bitstring(table[2], 24))

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
            res = bitand([tab[idx] for tab, idx in zip(tables, var)])
            results.append(res)
        return results

    @property
    def hist(self):
        hist = {}
        for res in self.table:
            found = bin(res).count('1')
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
            print(sym, bitstring(tab, 24), bin(tab).count('1'))

    def __repr__(self):
        return "Combination(%s)" % ', '.join(["'%s'" % w.choice for w in self.weighs])
