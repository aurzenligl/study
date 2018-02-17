from inspect import isclass
from functools import wraps

# Decorator implementation

def decorated(fun):
    desc = next((desc for desc in (staticmethod, classmethod)
                 if isinstance(fun, desc)), None)
    if desc:
        fun = fun.__func__

    @wraps(fun)
    def wrap(*args, **kwargs):
        cls, nonselfargs = _declassify(fun, args)
        clsname = cls.__name__ if cls else None
        print('class: %-10s func: %-15s args: %-10s kwargs: %-10s' %
              (clsname, fun.__name__, nonselfargs, kwargs))

    wrap.original = fun

    if desc:
        wrap = desc(wrap)
    return wrap

def _declassify(fun, args):
    if len(args):
        met = getattr(args[0], fun.__name__, None)
        if met:
            wrap = getattr(met, '__func__', None)
            if wrap.original is fun:
                maybe_cls = args[0]
                cls = maybe_cls if isclass(maybe_cls) else maybe_cls.__class__
                return cls, args[1:]
    return None, args

# Decorated code

@decorated
def simplefun():
    pass

class Class(object):
    @decorated
    def __init__(self):
        pass

    @decorated
    def method(self, a, b):
        pass

    @decorated
    @staticmethod
    def staticmethod(a1, a2=None):
        pass

    @decorated
    @classmethod
    def classmethod(cls):
        pass

# Calling decorated functions

simplefun()
instance = Class()
instance.method(1, 2)
instance.staticmethod(a1=3)
instance.classmethod()
Class.staticmethod(a1=3)
Class.classmethod()
