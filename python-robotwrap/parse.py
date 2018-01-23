#!/usr/bin/env python

import os
import sys
import ast

def is_fun(node):
    return isinstance(node, ast.FunctionDef)

def is_class(node):
    return isinstance(node, ast.ClassDef)

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
        if is_class(child) and child.name == name:
            child.parent = node
            return child
    return None

def find_functions(node):
    funs = [child for child in ast.iter_child_nodes(node) if is_fun(child)]
    for fun in funs:
        fun.parent = node
    return funs

def insert_decorators(path, locs):
    with open(path, 'rw+') as f:
        lines = f.readlines()
        already_inserted = 0
        for loc in locs:
            line, offset = loc
            pill = ' ' * offset + '@trxstub\n'
            lines.insert(line + already_inserted -1, pill)
            already_inserted += 1
        f.seek(0)
        f.truncate()
        f.write(''.join(lines))

def to_locname(node):
    path = '::'.join([p.name for p in get_path(node)])
    return '%s:%s:%s' % (path, node.lineno, node.col_offset)

def to_locname2(path, locs):
    insert_decorators(path, locs)

def sort_by_line(nodes):
    return sorted(nodes, key=lambda x: x.lineno)

def file_to_locs(filepath):
    tree = ast.parse(open(filepath).read())
    tree.name = filepath

    def to_purebasename(filepath):
        return os.path.splitext(os.path.basename(filepath))[0]

    funs = find_functions(tree)
    meths = find_functions(find_class(tree, to_purebasename(filepath)))

    keywords = sort_by_line(funs + meths)
    insert_decorators(filepath, [(node.lineno, node.col_offset) for node in keywords])
    return map(to_locname, keywords)

def main():
    args = sys.argv[1:]
    if len(args) != 1:
        sys.exit('Usage: parse.py <filename>')
    filepath = sys.argv[1]
    for loc in file_to_locs(filepath):
        print loc

if __name__ == '__main__':
    main()

