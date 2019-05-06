#!/usr/bin/env python2

import os
import sys
import itertools as it
from rospkg import RosPack

def abspathify(path):
    assert os.path.isdir(path), '"%s" is not a directory' % path
    return os.path.abspath(path)

def is_chosen(rp, pkg, chosen):
    return any(rp.get_path(pkg).startswith(ch) for ch in chosen)

def which_chosen(rp, pkg, chosen):
    return next(i for i, ch in enumerate(chosen) if rp.get_path(pkg).startswith(ch))

def get_deps(rp, pkg, packages):
    deps = rp.get_depends(pkg, False)
    deps = [d for d in deps if d in packages]
    return deps

def get_deplist(rp, packages):
    return [(pkg, which_chosen(rp, pkg, chosen), get_deps(rp, pkg, packages)) for pkg in packages]

def make_dot(deplist):
    def node(dot, n):
        dot.append(str(n))

    def edge(dot, n1, n2):
        dot.append('%s -> %s' % (n1, n2))

    def wrap(dot, pre, post):
        dot = [' ' * 4 + line for line in dot]
        dot = [pre] + dot + [post]
        return dot

    by_choice = lambda x: x[1]
    groups = it.groupby(sorted(deplist, key=by_choice), key=by_choice)

    graph = []

    for group in groups:
        dot = []
        for pkg, choice, deps in group[1]:
            node(dot, pkg)
            for dep in deps:
                edge(dot, pkg, dep)
        if graph:
            graph.append('')
        graph.extend(wrap(dot, 'subgraph cluster_%s {' % group[0], '}'))

    graph = wrap(graph, 'digraph {', '}')
    return '\n'.join(graph)

chosen = [abspathify(x) for x in sys.argv[1:]]
rp = RosPack(RosPack().ros_paths + chosen)
packages = [x for x in rp.list() if is_chosen(rp, x, chosen)]
deplist = get_deplist(rp, packages)
dot = make_dot(deplist)
print(dot)
