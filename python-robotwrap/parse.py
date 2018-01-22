#!/usr/bin/env python

import os
import sys
import ast

def is_fun(node):
    return isinstance(node, ast.FunctionDef)

def is_module(node):
    return isinstance(node, ast.FunctionDef)

def get_path(node):
    path = [node]
    while True:
        node = getattr(node, 'parent', None)
        if node:
            path.insert(0, node)
        else:
            return path

def find_class(node, name):
    for child in ast.iter_child_nodes(node):
        if isinstance(child, ast.ClassDef) and child.name == name:
            child.parent = node
            return child
    return None

def find_functions(node):
    funs = [child for child in ast.iter_child_nodes(node) if is_fun(child)]
    for fun in funs:
        fun.parent = node
    return funs

def to_locname(node):
    path = '::'.join([p.name for p in get_path(node)])
    return '%s:%s:%s' % (path, node.lineno, node.col_offset)

def sort_by_line(nodes):
    return sorted(nodes, key=lambda x: x.lineno)

def file_to_locs(filepath):
    tree = ast.parse(open(filepath).read())
    tree.name = filepath

    def to_purebasename(filepath):
        return os.path.splitext(os.path.basename(filepath))[0]

    funs = find_functions(tree)
    meths = find_functions(find_class(tree, to_purebasename(filepath)))

    return map(to_locname, sort_by_line(funs + meths))

def main():
    args = sys.argv[1:]
    if len(args) != 1:
        sys.exit('Usage: parse.py <filename>')
    filepath = sys.argv[1]
    for loc in file_to_locs(filepath):
        print loc

if __name__ == '__main__':
    main()

