#!/usr/bin/env python

import sys
from pathlib import Path

def parse_from_file(fname: Path):
    lines = fname.read_text().strip().splitlines()
    assert len(lines) == 5, f'expected 5 lines, got {len(lines)}'
    for i, line in enumerate(lines):
        assert len(line) == 5, f'expected line {i} to have length 5, got {len(line)}'
    for x in range(5):
        for y in range(5):
            assert lines[y][x] in 'abcd+.', f'expected characters from set "abcd+.", got "{lines[y][x]}"'

    def expected_block(kind):
        return ''.join(c if c in kind + '\n' else '.' for c in ''.join(l + '\n' for l in lines))

    def filtered_block(board, kind):
        return ''.join(c if c in kind + '\n' else '.' for c in str(board))

    def deduce_cats(board: Board):
        for y, l in enumerate(reversed(lines)):
            for x in range(5):
                if l[x] == '+':
                    board.place_cat(x, y)
        return board

    def deduce_block(board: Board, kind: str):
        exp = expected_block(kind)
        placements = [(x, y, t) for x in range(5) for y in range(5) for t in range(4)]
        for p in placements:
            b = board.clone()
            if b.place_block(*p, kind):
                if exp == filtered_block(b, kind):
                    return b
        raise Violation(f'block "{kind}" not found')

    catboard = deduce_cats(Board())
    startboard = catboard.clone()
    for kind in 'abcd':
        startboard = deduce_block(startboard, kind)

    return catboard, startboard

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

    def one_block_diff(self, other):
        diffs = 0
        for a, b in zip(self.history, other.history):
            diffs += a != b
        return diffs == 1

    def clone(self):
        return Board(self.occ, self.history)

    def __repr__(self):
        lines = []
        for y in reversed(range(5)):
            lines.append(''.join(self.occ.get((x, y), '.') for x in range(5)))
        return ''.join(l + '\n' for l in lines)

### problems ###

fname = Path(__file__).with_name('problem') / sys.argv[1]
catboard, startboard = parse_from_file(fname)

### calculate all boards ###

sols = []
placements = [(x, y, t) for x in range(5) for y in range(5) for t in range(4)]
for p1 in placements:
    a = catboard.clone()
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
                                sols.append(d)

### shortest path ###

import networkx as nx

g = nx.Graph()
for i, s in enumerate(sols):
    g.add_node(s)
for i, s in enumerate(sols):
    for j, t in enumerate(sols):
        if j > i:
            if s.one_block_diff(t):
                g.add_edge(s, t)

start = next(s for s in sols if s.occ == startboard.occ)
finish = next(s for s in sols if '+' not in str(s))
shortest_path = nx.shortest_path(g, start, finish)
connected_boards = nx.node_connected_component(g, start)

print_boards(sols)
print(f'boards: {len(sols)}')
print(f'connected boards: {len(connected_boards)}')
print(f'shortest path boards: {len(shortest_path)} ({sols.index(start)} -> {sols.index(finish)})')
print()
print_boards(shortest_path)

### dotting ###

import graphviz
dot = graphviz.Graph(engine='neato', graph_attr=dict(overlap='false', sep='+15'))

for i, s in enumerate(sols):
    kw = {}
    if s in shortest_path:
        kw['color'] = kw['fontcolor'] = 'blue'
        kw['penwidth'] = '5'
    if s is start:
        kw['color'] = kw['fontcolor'] = 'red'
        kw['penwidth'] = '5'
    if s is finish:
        kw['color'] = kw['fontcolor'] = 'darkgreen'
        kw['penwidth'] = '5'
    if s in connected_boards:
        dot.node(str(i), str(s), shape='box', fontname='monospace', **kw)

for i, s in enumerate(sols):
    for j, t in enumerate(sols):
        if j > i:
            if s.one_block_diff(t):
                kw = {}
                if s in shortest_path and t in shortest_path:
                    kw['color'] = 'blue'
                    kw['penwidth'] = '5'
                if s in connected_boards:
                    dot.edge(str(i), str(j), **kw)

dot.render('solution.gv')
