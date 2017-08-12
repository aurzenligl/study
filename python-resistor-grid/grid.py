#!/usr/bin/env python

import graphviz as gv

class Graphizer:
    def __init__(self, name):
        self.count = 0
        self.name = name
    def __call__(self, nodes, edges):
        g = gv.Graph(format='svg', engine='neato')
        for n in nodes:
            g.node(n)
        for e in edges:
            g.edge(*e)
        g.render(filename='%s.%03d' % (self.name, self.count))
        self.count += 1
    def from_graph(self, graph):
        nodes = [node.name for node in graph.nodes]
        edges = {neigh.edge: (neigh.node.name, node.name) for node in graph.nodes for neigh in node.neighbors}
        edges = edges.values()
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
        self.nonremovable = False
    @property
    def name(self):
        return '%s,%s' % (self.loc.x, self.loc.y)
    def __str__(self):
        return self.name + '\n' + ''.join(['-> ' + n.node.name + '\n' for n in self.neighbors])

class Graph:
    def __init__(self):
        self.nodes = []
    def find(self, x, y):
        return next((node for node in self.nodes if node.loc == Loc(x, y)), None)
    def add(self, x, y):
        node = Node(Loc(x, y))
        self.nodes.append(node)
        self._link(node)
    def nonremovable(self, x, y):
        node = self.find(x, y)
        if not node:
            raise Exception('node not found')
        node.nonremovable = True
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

# TODO label edge values
# TODO set fixed node coordinates

def reduce_serial(gr):
    def find_serial(gr):
        for node in gr.nodes:
            if not node.nonremovable:
                if len(node.neighbors) == 2:
                    return node
    node = find_serial(gr)
    if node:
        gr.nodes.remove(node)
        ne1, ne2 = node.neighbors
        no1 = ne1.node
        no2 = ne2.node
        nb1 = next(nb for nb in no1.neighbors if nb.node is node)  # TODO neighbor.other_neighbor
        nb2 = next(nb for nb in no2.neighbors if nb.node is node)
        nb1.edge.value += nb2.edge.value
        nb2.edge = nb1.edge
        nb2.node = no1
        nb1.node = no2
        return nb2.edge

x = Graph()
for addr in [(i, j) for i in range(0, 3) for j in range(0, 2)]:
    x.add(*addr)
x.nonremovable(0, 0)
x.nonremovable(2, 1)
g.from_graph(x)
reduce_serial(x)
g.from_graph(x)
reduce_serial(x)
g.from_graph(x)

# x = Graph()
# for addr in [(i, j) for i in range(-1, 4) for j in range(-1, 3)]:
#     x.add(*addr)
# g.from_graph(x)
# 
# x = Graph()
# for addr in [(i, j) for i in range(-2, 5) for j in range(-2, 4)]:
#     x.add(*addr)
# g.from_graph(x)

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

