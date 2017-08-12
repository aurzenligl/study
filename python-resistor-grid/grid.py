#!/usr/bin/env python

import graphviz as gv
from fractions import Fraction as frac

class Graphizer:
    def __init__(self, name):
        self.count = 0
        self.name = name

    def __call__(self, nodes, edges):
        g = gv.Graph(format='svg', engine='neato')
        for n in nodes:
            g.node(n[0], **n[1])
        for e in edges:
            g.edge(*e[0], **e[1])
        g.render(filename='%s.%03d.graph' % (self.name, self.count))
        self.count += 1

    def from_graph(self, graph):
        nodes = [(node.name, {'pos': '%s,%s!' % (node.loc[0] * 1, node.loc[1] * 1)}) for node in graph.nodes]
        edges = [((edge.nodes[0].name, edge.nodes[1].name), {'label': str(edge.value)}) for edge in graph.edges]
        self(nodes, edges)

class Node:
    def __init__(self, loc):
        self.loc = loc
        self.nonremovable = False

    @property
    def name(self):
        return '%s,%s' % self.loc

    def __repr__(self):
        return "<Node '%s'>" % self.name

class Edge:
    def __init__(self, n1, n2, value):
        self.nodes = (n1, n2)
        self.value = value

    def has(self, node):
        return node in self.nodes

    def __repr__(self):
        return "<Edge '%s':'%s'='%s'>" % (self.nodes + (self.value,))

class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def find(self, loc):
        return next((node for node in self.nodes if node.loc == loc), None)

    def add_node(self, loc):
        node = Node(loc)
        self.nodes.append(node)
        self._link(node)

    def add_edge(self, edge):
        self.edges.append(edge)

    def remove_node(self, node):
        self.nodes.remove(node)

    def remove_edge(self, edge):
        self.edges.remove(edge)

    def set_nonremovable(self, loc):
        node = self.find(loc)
        if not node:
            raise Exception('node not found')
        node.nonremovable = True

    def neighbors(self, node):
        return [edge for edge in self.edges if edge.has(node)]

    def _link(self, node):
        candidates = [
            self.find((node.loc[0]+1, node.loc[1])),
            self.find((node.loc[0]-1, node.loc[1])),
            self.find((node.loc[0], node.loc[1]+1)),
            self.find((node.loc[0], node.loc[1]-1)),
        ]
        for cand in candidates:
            if cand:
                self.edges.append(Edge(node, cand, frac(1)))

g = Graphizer('snapshot')

def reduce_serial(gr):
    def find_serial(gr):
        for node in gr.nodes:
            if not node.nonremovable:
                nedges = gr.neighbors(node)
                if len(nedges) == 2:
                    return node, nedges

    def merge_serial_edges(node, e1, e2):
        merged_nodes = [_ for _ in e1.nodes + e2.nodes if _ is not node]
        merged_value = e1.value + e2.value
        return Edge(*merged_nodes, value=merged_value)

    res = find_serial(gr)
    if res:
        node, (e1, e2) = res
        gr.remove_node(node)
        gr.remove_edge(e1)
        gr.remove_edge(e2)
        e3 = merge_serial_edges(node, e1, e2)
        gr.add_edge(e3)
        return node, e1, e2

def reduce_parallel(gr):
    pass

def reduce_wye(gr):
    def find_wye(gr):
        for node in gr.nodes:
            if not node.nonremovable:
                nedges = gr.neighbors(node)
                if len(nedges) == 3:
                    return node, nedges

    def to_delta(node, e1, e2, e3):
        n1 = next(_ for _ in e1.nodes if _ is not node)
        n2 = next(_ for _ in e2.nodes if _ is not node)
        n3 = next(_ for _ in e3.nodes if _ is not node)
        numer = e1.value * e2.value + e2.value * e3.value + e3.value * e1.value
        ea = Edge(n2, n3, value=numer/e1.value)
        eb = Edge(n3, n1, value=numer/e2.value)
        ec = Edge(n1, n2, value=numer/e3.value)
        return ea, eb, ec

    res = find_wye(gr)
    if res:
        node, (e1, e2, e3) = res
        gr.remove_node(node)
        gr.remove_edge(e1)
        gr.remove_edge(e2)
        gr.remove_edge(e3)
        ea, eb, ec = to_delta(node, e1, e2, e3)
        gr.add_edge(ea)
        gr.add_edge(eb)
        gr.add_edge(ec)
        return node, e1, e2, e3

x = Graph()
for loc in [(i, j) for i in range(0, 3) for j in range(0, 2)]:
    x.add_node(loc)
x.set_nonremovable((0, 0))
x.set_nonremovable((2, 1))
g.from_graph(x)
assert reduce_serial(x)
g.from_graph(x)
assert reduce_serial(x)
g.from_graph(x)
assert reduce_wye(x)
g.from_graph(x)

#x = Graph()
#for loc in [(i, j) for i in range(-1, 4) for j in range(-1, 3)]:
#    x.add_node(loc)
#g.from_graph(x)
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

