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
        nodes = [node.name for node in graph.nodes]
        edges = [(neigh.node.name, node.name) for node in graph.nodes for neigh in node.neighbors]
        self(nodes, edges)

class Loc:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __eq__(self, rhs):
        return self.x == rhs.x and self.y == rhs.y

class Edge:
    def __init__(self, value):
        self.value = value

class Neighbor:
    def __init__(self, node, edge):
        self.node = node
        self.edge = edge

class Node:
    def __init__(self, loc):
        self.loc = loc
        self.neighbors = []
    @property
    def name(self):
        return '%s,%s' % (self.loc.x, self.loc.y)

class Graph:
    def __init__(self):
        self.nodes = []
    def find(self, x, y):
        return next((node for node in self.nodes if node.loc == Loc(x, y)), None)
    def add(self, x, y):
        node = Node(Loc(x, y))
        self.nodes.append(node)
        self._link(node)
    def _link(self, node):
        candidates = [
            self.find(node.loc.x+1, node.loc.y),
            self.find(node.loc.x-1, node.loc.y),
            self.find(node.loc.x, node.loc.y+1),
            self.find(node.loc.x, node.loc.y-1),
        ]
        for cand in candidates:
            if cand:
                edge = Edge(1)
                cand.neighbors.append(Neighbor(node, edge))
                node.neighbors.append(Neighbor(cand, edge))

g = Graphizer('snapshot')

# TODO loop over making nodes
# TODO label edge values
# TODO single edges instead of doubles

x = Graph()
x.add(0, 0)
x.add(0, 1)
x.add(1, 0)
x.add(1, 1)
g.from_graph(x)

x = Graph()
x.add(0, 0)
x.add(0, 1)
x.add(1, 0)
x.add(1, 1)
x.add(2, 0)
x.add(2, 1)
g.from_graph(x)

x = Graph()
x.add(-1, -1)
x.add(-1, 0)
x.add(-1, 1)
x.add(-1, 2)
x.add(0, -1)
x.add(0, 0)
x.add(0, 1)
x.add(0, 2)
x.add(1, -1)
x.add(1, 0)
x.add(1, 1)
x.add(1, 2)
x.add(2, -1)
x.add(2, 0)
x.add(2, 1)
x.add(2, 2)
x.add(3, -1)
x.add(3, 0)
x.add(3, 1)
x.add(3, 2)
g.from_graph(x)

'''
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
'''

