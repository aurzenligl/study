#!/usr/bin/env python

import inspect

class X(object):
    def __init__(self, name):
        self.name = name
    def foo(self, x):
        return ':'.join((self.name, x))

class Y(object):
    def bar(self, x, y):
        return ':'.join((self.name, x, y))

def monkey_patch_methods(obj, Type):
    for name, value in inspect.getmembers(Type, predicate=inspect.ismethod):
        setattr(obj, name, value.__get__(obj))

x = X('inst')
monkey_patch_methods(x, Y)
print(x.foo('abc'))
print(x.bar('1', '2'))

y = X('sec')
monkey_patch_methods(y, Y)
print(y.foo('abc'))
print(y.bar('1', '2'))
