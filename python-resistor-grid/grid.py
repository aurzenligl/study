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
    def find_parallel(gr):
        x = {}
        for edge in gr.edges:
            y = x.setdefault(tuple(sorted(edge.nodes)), [])
            if not y:
                y.append(edge)
            else:
                return y[0], edge

    def merge_parallel(e1, e2):
        merged_value = (e1.value * e2.value) / (e1.value + e2.value)
        return Edge(*e1.nodes, value=merged_value)

    res = find_parallel(gr)
    if res:
        e1, e2 = res
        gr.remove_edges([e1, e2])
        e3 = merge_parallel(e1, e2)
        gr.add_edge(e3)
        return e1, e2

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
    from_graph(printer, gr)
    while True:
        while reduce_parallel(gr):
            pass
        from_graph(printer, gr)
        if reduce_star(gr):
            from_graph(printer, gr)
            continue
        break

g = Graphizer('snapshot')

x = Graph()
for loc in [(i, j) for i in range(0, 3) for j in range(0, 2)]:
    x.add_node(loc)
x.set_nonremovable((0, 0))
x.set_nonremovable((2, 1))
reduce(x, g)
assert len(x.edges) == 1
print x.edges[0].value

# x = Graph()
# for loc in [(i, j) for i in range(-1, 4) for j in range(-1, 3)]:
#     x.add_node(loc)
# x.set_nonremovable((0, 0))
# x.set_nonremovable((2, 1))
# reduce(x, g)

# x = Graph()
# for loc in [(i, j) for i in range(-2, 5) for j in range(-2, 4)]:
#     x.add_node(loc)
# x.set_nonremovable((0, 0))
# x.set_nonremovable((2, 1))
# reduce(x, g)

# def gen_reduced(level):
#     x = Graph()
#     for loc in [(i, j) for i in range(-level, 3 + level) for j in range(-level, 2 + level)]:
#         x.add_node(loc)
#     x.set_nonremovable((0, 0))
#     x.set_nonremovable((2, 1))
#     reduce(x)
#     assert len(x.edges) == 1
#     return x.edges[0].value
#
# for i in range(10):
#     val = gen_reduced(i)
#     print i, val, float(val)
