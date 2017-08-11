#!/usr/bin/env python

import graphviz as gv

class Graphizer:
    def __init__(self, name):
        self.count = 0
        self.name = name
    def __call__(self, nodes, edges):
        g = gv.Graph(format='svg')
        for n in nodes:
            g.node(n)
        for e in edges:
            g.edge(*e)
        g.render(filename='%s.%03d' % (self.name, self.count))
        self.count += 1
    def from_graph(self, graph):
        nodes = ['%s:%s' % (node.loc.x, node.loc.y) for node in graph.nodes]
        self(nodes, [])

class Loc:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Node:
    def __init__(self, loc):
        self.loc = loc
        self.nodes = []

class Graph:
    def __init__(self):
        self.nodes = []
    def add(self, x, y):
        self.nodes.append(Node(Loc(x, y)))

# 01 11 21
# 00 10 20

g = Graphizer('snapshot')

x = Graph()
x.add(0, 0)
x.add(0, 1)
x.add(1, 0)

g.from_graph(x)

g([
    'A',
    'B',
    'C',
], [
    ('B', 'C'),
])
g([
    'A',
    'B',
    'C',
], [
    ('B', 'C'),
    ('A', 'B'),
    ('A', 'A'),
])
g([
], [
    ('B', 'C'),
    ('A', 'B'),
    ('A', 'A'),
    ('D', 'B'),
    ('D', 'A'),
])

