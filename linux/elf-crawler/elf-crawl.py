#!/usr/bin/env python

import os
import sys
import functools
from pathlib import Path
from elftools.elf.elffile import ELFFile
from elftools.elf.elffile import DynamicSection

assert len(sys.argv) == 2, 'expected elf path'
path = sys.argv[1]

@functools.lru_cache
def get_libs(path: str):
    libs = {}
    for fn in os.listdir(path):
        if fn.startswith('lib') and '.so' in fn:
            libs[fn] = path + '/' + fn
    return libs

# LD_LIBRARY_PATH

@functools.lru_cache
def get_ldlibrarypaths():
    paths = []
    for path in os.getenv('LD_LIBRARY_PATH').split(':'):
        if os.path.isdir(path):
            paths.append(path)
    return paths

@functools.lru_cache
def get_libs_global():
    libs = {}
    import subprocess
    lines = subprocess.check_output('ldconfig -p | sed "s/(.*)//g"', shell=True).decode().splitlines()
    for x in lines[1:]:
        a, b = x.split(' => ')
        a = a.strip()
        b = b.strip()
        libs.setdefault(a.strip(), b.strip())
    return libs

@functools.lru_cache
def from_rpath(elf, name):
    for path in elf_rpaths(elf):
        for fn in os.listdir(path):
            if fn == name:
                return f'{path}/{fn}'

@functools.lru_cache
def from_runpath(elf, name):
    for path in elf_runpaths(elf):
        for fn in os.listdir(path):
            if fn == name:
                return f'{path}/{fn}'

def from_ldlibrarypath(name):
    for path in get_ldlibrarypaths():
        for fn in os.listdir(path):
            if fn == name:
                return f'{path}/{fn}'

def from_ldconfig(name):
    return get_libs_global().get(name)

def elf_rpaths(elf: ELFFile):
    paths = []
    for tag in elf.get_section_by_name('.dynamic').iter_tags():
        if tag.entry.d_tag == 'DT_RPATH':
            for path in tag.rpath.split(':'):
                if path:
                    paths.append(path)
    return paths

@functools.lru_cache
def elf_runpaths(elf: ELFFile):
    paths = []
    for tag in elf.get_section_by_name('.dynamic').iter_tags():
        if tag.entry.d_tag == 'DT_RUNPATH':
            for path in tag.runpath.split(':'):
                if path and os.path.isdir(path):
                    paths.append(path)
    return paths

@functools.lru_cache
def elf_needed(elf: ELFFile):
    libnames = []
    for tag in elf.get_section_by_name('.dynamic').iter_tags():
        if tag.entry.d_tag == 'DT_NEEDED':
            libnames.append(tag.needed)
    return libnames

@functools.lru_cache
def elf_deps(elf: ELFFile):
    deps = []
    for name in elf_needed(elf):
        path = None
        path = path or from_rpath(elf, name)
        path = path or from_ldlibrarypath(name)
        path = path or from_runpath(elf, name)
        path = path or from_ldconfig(name)
        assert path, f'cannot resolve library named: {name}'
        deps.append(path)
    return deps

@functools.lru_cache
def resolve_deps(path: str):
    elf = ELFFile.load_from_path(path)
    elf.path = path
    return elf_deps(elf)

visited = set()

def dig(path: str):
    visited.add(path)
    print(f'dig: {path}')
    for dep in resolve_deps(path):
        if dep not in visited:
            dig(dep)

dig(path)




# queue = [path]
# visited = set()
# while queue:
#     p = queue.pop(0)
#     if p in visited:
#         continue
#     print(f'dig: {p}')
#     visited.add(p)
#     deps = resolve_deps(p)
#     for d in deps:
#         queue.append(d)
#         print(f'    dep: {d}')









    # for d in deps:
    #     dig(d, visited)

# dig(path, visited)

# import pdb;pdb.set_trace()

# for x in libs:
#     print('[libs]', x, libs[x])

# TODO: read runpath tag
# TODO: get all needed libs



# runpath(elf)

        # print(tag)
        # tag.runpath
        # import pdb;pdb.set_trace()

    # print(tag.entry.d_tag)
    # print(tag)
    # print(type(tag))


# DT_RUNPATH
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED
# DT_NEEDED


# from ctypes.util import find_library
# print(find_library('roscpp'))
# print(find_library('aeolus_core_portable'))



# print(find_library('jpeg.so'))

# for a in x.iter_sections():
#     # import pdb;pdb.set_trace()
#     print('section', '|', a.name, '|', a)
#     # if a.name == '.dynamic':
#     #     import pdb;pdb.set_trace()
#     #     pass

# elftools.elf.dynamic.DynamicSection

# for a in x.iter_segments():
#     pass
#     # import pdb;pdb.set_trace()
#     # print('section', '|', a.name, '|', a)

# ".dynamic"

# for section in elf.iter_sections():
#     if isinstance(section, DynamicSection):
#         print(section.name)
#         print('XXXXXXX')
#         for tag in section.iter_tags():
#             print(tag)
#             # print(tag.entry.d_tag, tag.needed)

# # import pdb;pdb.set_trace()
