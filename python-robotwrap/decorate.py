#!/usr/bin/env python

import os
import sys
import ast

def is_fun(node):
    return isinstance(node, ast.FunctionDef)

def is_class(node):
    return isinstance(node, ast.ClassDef)

def find_class(node, name):
    for child in ast.iter_child_nodes(node):
        if is_class(child) and child.name == name:
            child.parent = node
            return child
    return None

def find_functions(node):
    if node is None:
        return []
    funs = [child for child in ast.iter_child_nodes(node) if is_fun(child)]
    for fun in funs:
        fun.parent = node
    return funs

def insert(func_decorator, class_decorator, path, nodes):
    with open(path, 'rw+') as f:
        lines = f.readlines()
        already_inserted = 0
        for node in nodes:
            line, offset = (node.lineno, node.col_offset)
            if not is_class(node.parent):
                pill = ' ' * offset + func_decorator + '\n'
            else:
                pill = ' ' * offset + class_decorator + '\n'
            lines.insert(line + already_inserted -1, pill)
            already_inserted += 1
        f.seek(0)
        f.truncate()
        f.write(''.join(lines))

def sort_by_line(nodes):
    return sorted(nodes, key=lambda x: x.lineno)

def process_file(filepath):
    tree = ast.parse(open(filepath).read())
    tree.name = filepath

    def to_purebasename(filepath):
        return os.path.splitext(os.path.basename(filepath))[0]

    funs = find_functions(tree)
    meths = find_functions(find_class(tree, to_purebasename(filepath)))

    insert('@trxstub(is_fun=True)', '@trxstub(is_fun=False)', filepath, sort_by_line(funs + meths))

def main():
    args = sys.argv[1:]
    if len(args) != 1:
        sys.exit('Usage: parse.py <filename>')
    filepath = sys.argv[1]
    process_file(filepath)

if __name__ == '__main__':
    main()

