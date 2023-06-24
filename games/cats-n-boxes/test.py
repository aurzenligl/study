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
        Aaa
        """
        fn(x, y)
        fn(*translate(x, y, *rotate(0, 1, th)))
        fn(*translate(x, y, *rotate(1, 0, th)))
        fn(*translate(x, y, *rotate(2, 0, th)))

    @staticmethod
    def b(x, y, th, fn):
        """
        ..b
        Bbb
        """
        fn(x, y)
        fn(*translate(x, y, *rotate(0, 1, th)))
        fn(*translate(x, y, *rotate(0, 2, th)))
        fn(*translate(x, y, *rotate(1, 2, th)))

    @staticmethod
    def c(x, y, th, fn):
        """
        .c.
        Ccc
        """
        fn(x, y)
        fn(*translate(x, y, *rotate(0, 1, th)))
        fn(*translate(x, y, *rotate(0, 2, th)))
        fn(*translate(x, y, *rotate(1, 1, th)))

    @staticmethod
    def d(x, y, th, fn):
        """
        ..d
        Ddd
        .d.
        """
        fn(x, y)
        fn(*translate(x, y, *rotate(0, 1, th)))
        fn(*translate(x, y, *rotate(0, 2, th)))
        fn(*translate(x, y, *rotate(1, 2, th)))
        fn(*translate(x, y, *rotate(-1, 1, th)))

class Board:
    def __init__(self):
        self.occ = {}

    def place_block(self, col, row, theta, kind):
        def pop(x, y):
            if x < 0 or x > 4 or y < 0 or y > 4:
                raise Violation('out of bounds')
            if (x, y) in self.occ:
                raise Violation('collision with something')
            self.occ[(x, y)] = kind
        getattr(Gen, kind)(col, row, theta, pop)

    def __repr__(self):
        lines = []
        for y in reversed(range(5)):
            lines.append(''.join(self.occ.get((x, y), '.') for x in range(5)))
        return ''.join(l + '\n' for l in lines)

placements = [(x, y, t) for x in range(5) for y in range(5) for t in range(4)]

v = []
for p1 in placements:
    for p2 in placements:
        for p3 in placements:
            for p4 in placements:
                try:
                    a = Board()
                    a.place_block(*p1, 'a')
                    a.place_block(*p2, 'b')
                    a.place_block(*p3, 'c')
                    a.place_block(*p4, 'd')
                    v.append(a)
                except Violation as e:
                    pass
print_boards(v)

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
