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

def from_graph(printer, graph):
    if printer:
        printer.from_graph(graph)

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

    def add_edges(self, edges):
        for edge in edges:
            self.add_edge(edge)

    def remove_node(self, node):
        self.nodes.remove(node)

    def remove_edge(self, edge):
        self.edges.remove(edge)

    def remove_edges(self, edges):
        for edge in edges:
            self.remove_edge(edge)

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

def reduce_parallel(gr):
    def find_parallel(gr):
        x = {}
        for edge in gr.edges:
            y = x.setdefault(tuple(sorted(edge.nodes)), [])
            y.append(edge)
        for _, edges in x.items():
            if len(edges) > 1:
                yield edges

    def merge_parallel(edges):
        assert len(edges) == 2
        merged_value = (edges[0].value * edges[1].value) / (edges[0].value + edges[1].value)
        return Edge(*edges[0].nodes, value=merged_value)

    for edges in find_parallel(gr):
        gr.remove_edges(edges)
        gr.add_edge(merge_parallel(edges))

def reduce_star(gr):
    def find_star(gr):
        for node in gr.nodes:
            if not node.nonremovable:
                nedges = gr.neighbors(node)
                if len(nedges) > 1:
                    return node, nedges

    def to_delta(node, es):
        ns = [next(_ for _ in e.nodes if _ is not node) for e in es]
        ens = zip(es, ns)
        suminv = sum([1 / e.value for e in es])
        return [Edge(n1, n2, value=e1.value*e2.value*suminv) for e1, n1 in ens for e2, n2 in ens if id(e1) < id(e2)]

    res = find_star(gr)
    if res:
        node, nedges = res
        gr.remove_node(node)
        gr.remove_edges(nedges)
        gr.add_edges(to_delta(node, nedges))
        return node, nedges

def reduce(gr, printer=None):
    while True:
        reduce_parallel(gr)
        from_graph(printer, gr)
        if reduce_star(gr):
            continue
        break

def gen_reduced(level):
    x = Graph()
    for loc in [(i, j) for i in range(-level, 3 + level) for j in range(-level, 2 + level)]:
        x.add_node(loc)
    x.set_nonremovable((0, 0))
    x.set_nonremovable((2, 1))
    reduce(x)
    assert len(x.edges) == 1
    return x.edges[0].value

g = Graphizer('snapshot')

# x = Graph()
# lev = 0
# for loc in [(i, j) for i in range(0-lev, 3+lev) for j in range(0-lev, 2+lev)]:
#     x.add_node(loc)
# x.set_nonremovable((0, 0))
# x.set_nonremovable((2, 1))
# reduce(x, g)
# assert len(x.edges) == 1
# print x.edges[0].value

# TODO profile performance and optimize
# TODO edges may be held as dictionary: tuple of nodes may be the key

for i in range(10):
    val = gen_reduced(i)
    print i, val, float(val)
