def to_cls(f):
    cls = getattr(f, 'im_class', None)
    if cls == type:
        cls = f.im_self
    return cls

def to_path(f):
    cls = to_cls(f)
    physical_path = '%s:%s' % (f.func_code.co_filename, f.func_code.co_firstlineno)
    logical_path = (cls.__name__ + '.' if cls else '') + f.func_name
    return '%s %s' % (physical_path, logical_path)

class decorated(object):
    def __init__(self, func):
        if isinstance(func, staticmethod):
            func.__func__.is_static = True
            self.func = classmethod(func.__func__)
        else:
            self.func = func

    def __get__(self, obj, type=None):
        return self.__class__(self.func.__get__(obj, type))

    def __call__(self, *args, **kwargs):
        print to_path(self.func), args, kwargs

        if getattr(self.func, 'is_static', None):
            f = self.func.__func__
        else:
            f = self.func
        return f(*args, **kwargs)

import __builtin__

__builtin__.decorated = decorated
