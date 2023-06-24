#!/usr/bin/env python

# TODO: introduce rot

# def rot(x, y, th):
#     if th == 0:
#         return x, y
#     elif th == 1:
#         return

def repr_occ(o):
    lines = []
    for y in reversed(range(5)):
        lines.append(''.join(o.get((x, y), '.') for x in range(5)))
    return ''.join(l + '\n' for l in lines)

def print_occs(occs):
    occs = occs[:]
    n = 32
    lines = []
    while len(occs) > n:
        for y in reversed(range(5)):
            lines.append(' '.join(''.join(o.get((x, y), '.') for x in range(5)) for o in occs[:n]))
        occs = occs[n:]
        if occs:
            lines.append('')
    if occs:
        for y in reversed(range(5)):
            lines.append(' '.join(''.join(o.get((x, y), '.') for x in range(5)) for o in occs))
    print(''.join(l + '\n' for l in lines))

def print_blocks(bls):
    print_occs([b.occ for b in bls])

class Violation(RuntimeError):
    pass

class Gen:
    @staticmethod
    def a(x, y, th, fn):
        """
        th0 th1 th2 th3
        a.. Aa. aaA .a
        Aaa a.. ..a .a
            a..     aA
        """
        fn(x, y)
        if th == 0:
            fn(x, y + 1)
            fn(x + 1, y)
            fn(x + 2, y)
        elif th == 1:
            fn(x + 1, y)
            fn(x, y - 1)
            fn(x, y - 2)
        elif th == 2:
            fn(x + 1, y)
            fn(x, y - 1)
            fn(x, y - 2)
        elif th == 3:
            fn(x - 1, y)
            fn(x, y + 1)
            fn(x, y + 2)

# TODO: class board

class Block:
    def __init__(self, x, y, kind, theta):
        self.x = x
        self.y = y
        self.k = kind
        self.th = theta

    @property
    def occ(self):
        m = {}
        def pop(x, y):
            if x < 0 or x > 4 or y < 0 or y > 4:
                raise Violation()
            m[(x, y)] = self.k
        getattr(Gen, self.k)(self.x, self.y, self.th, pop)
        return m

    def __bool__(self):
        try:
            self.occ
            return True
        except Violation:
            return False

    def __repr__(self):
        try:
            o = self.occ
            lines = []
            for y in reversed(range(5)):
                lines.append(''.join(o.get((x, y), '.') for x in range(5)))
            return ''.join(l + '\n' for l in lines)
        except Violation:
            return '+---+\n|vio|\n|lat|\n|ion|\n+---+\n'


bls = [Block(x, y, 'a', t) for x in range(5) for y in range(5) for t in range(4)]
valid = [b for b in bls if b]
print_blocks(valid)

v = []
for x in range(5):
    for y in range(5):
        for t in range(4):
            a = Block(x, y, 'a', t)
            if a:
                v.append(a)
print_blocks(v)

# print(a)

# a = Block(0, 0, 'a', 2)
# if a:
#     print(a)

# a = Block(3, 2, 'a', 2)
# print(a)

# import pdb;pdb.set_trace()

# a.occ

    # def __repr__():
    #     pass



# import graphviz
# dot = graphviz.Digraph(comment='The Round Table')

# dot.node('A', '''\
# +aaa.
# .b+a+
# bbdd.
# +bcdd
# ccCd+
# ''', fontname='monospace')

# dot.node('B', 'Sir Bedevere the Wise')
# dot.node('L', '', image='/home/aurzenligl/img.png')
# dot.edges(['AB', 'AL'])
# dot.edge('B', 'L', constraint='false')

# dot.render('doctest-output/round-table.gv').replace('\\', '/')

# print(dot.source)
#import pdb;pdb.set_trace()
#dot
