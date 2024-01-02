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

@functools.lru_cache
def get_libs_ldlibrarypath():
    libs = {}
    for path in reversed(os.getenv('LD_LIBRARY_PATH').split(':')):
        if not os.path.isdir(path):
            continue
        for fn in os.listdir(path):
            if fn.startswith('lib') and '.so' in fn:
                libs[fn] = path + '/' + fn
    return libs

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
    for path in reversed(elf_rpaths(elf)):
        return get_libs(path).get(name)

@functools.lru_cache
def from_runpath(elf, name):
    for path in reversed(elf_runpaths(elf)):
        return get_libs(path).get(name)

def from_ldlibrarypath(name):
    return get_libs_ldlibrarypath().get(name)

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
        print(f'    dep: {path} -> {dep}')
        if dep not in visited:
            dig(dep)

dig(path)
