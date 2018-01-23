import os
import sys

class helper(object):
    def dont_catch_this():
        pass

class Example2(object):
    def __init__(self):
        pass

    def foo(a, b):
        def irrelevant():
            pass
        pass

    class InnerClass(object):
        def method():
            pass

    class Example(object):
        def method():
            pass
