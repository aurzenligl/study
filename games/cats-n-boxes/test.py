#!/usr/bin/env python

def print_boards(boards):
    n = 32
    lines = []
    occs = [b.occ for b in boards]
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

class Violation(RuntimeError):
    pass

# TODO: introduce rot
def rotate(x, y, th):
    if th == 0:
        return x, y
    elif th == 1:
        return y, -x
    elif th == 2:
        return -x, -y
    elif th == 3:
        return -y, x
    else:
        raise Exception('unexpected theta: %s' % th)

def translate(x1, y1, x2, y2):
    return x1 + x2, y1 + y2

class Gen:
    @staticmethod
    def a(x, y, th, fn):
        """
        a..
        Axa
        """
        fn(x, y, False)
        fn(*translate(x, y, *rotate(1, 0, th)), True)
        fn(*translate(x, y, *rotate(2, 0, th)), False)
        fn(*translate(x, y, *rotate(0, 1, th)), False)

    @staticmethod
    def b(x, y, th, fn):
        """
        ..b
        Bbx
        """
        fn(x, y, False)
        fn(*translate(x, y, *rotate(1, 0, th)), False)
        fn(*translate(x, y, *rotate(2, 0, th)), True)
        fn(*translate(x, y, *rotate(2, 1, th)), False)

    @staticmethod
    def c(x, y, th, fn):
        """
        .c.
        Cxc
        """
        fn(x, y, False)
        fn(*translate(x, y, *rotate(1, 0, th)), True)
        fn(*translate(x, y, *rotate(2, 0, th)), False)
        fn(*translate(x, y, *rotate(1, 1, th)), False)

    @staticmethod
    def d(x, y, th, fn):
        """
        ..x
        Ddd
        .x.
        """
        fn(x, y, False)
        fn(*translate(x, y, *rotate(1, 0, th)), False)
        fn(*translate(x, y, *rotate(2, 0, th)), False)
        fn(*translate(x, y, *rotate(2, 1, th)), True)
        fn(*translate(x, y, *rotate(1, -1, th)), True)

class Board:
    def __init__(self, occ=None, history=None):
        self.occ = occ.copy() if occ else {}
        self.history = history.copy() if history else []

    def place_cat(self, col, row):
        if (col, row) in self.occ:
            raise Violation('collision with something')
        self.occ[(col, row)] = '+'

    def place_block(self, col, row, theta, kind):
        def pop(x, y, box):
            if x < 0 or x > 4 or y < 0 or y > 4:
                raise Violation('out of bounds')
            if (x, y) in self.occ:
                cat_in_a_box = box and self.occ[(x, y)] == '+'
                if not cat_in_a_box:
                    raise Violation('collision with something')
            self.occ[(x, y)] = kind
        try:
            self.history.append((col, row, theta, kind))
            getattr(Gen, kind)(col, row, theta, pop)
            return True
        except Violation:
            return False

    def clone(self):
        return Board(self.occ, self.history)

    def __repr__(self):
        lines = []
        for y in reversed(range(5)):
            lines.append(''.join(self.occ.get((x, y), '.') for x in range(5)))
        return ''.join(l + '\n' for l in lines)

placements = [(x, y, t) for x in range(5) for y in range(5) for t in range(4)]

vs = []
x = Board()
x.place_cat(0, 4)
x.place_cat(0, 1)
x.place_cat(2, 3)
x.place_cat(4, 3)
x.place_cat(4, 0)
print(x)
for p1 in placements:
    a = x.clone()
    if a.place_block(*p1, 'a'):
        for p2 in placements:
            b = a.clone()
            if b.place_block(*p2, 'b'):
                for p3 in placements:
                    c = b.clone()
                    if c.place_block(*p3, 'c'):
                        for p4 in placements:
                            d = c.clone()
                            if d.place_block(*p4, 'd'):
                                vs.append(d)
print_boards(vs)
print(f'solutions: {len(vs)}')
print()

for v in vs[:5] + vs[90:92]:
    print(v.history)
    print(v)

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
