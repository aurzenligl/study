def to_path(fun, cls):
    physical_path = '%s:%s' % (fun.func_code.co_filename, fun.func_code.co_firstlineno)
    logical_path = (cls.__name__ + '.' if cls else '') + fun.func_name
    return '%s %s' % (physical_path, logical_path)

class decorated(object):
    def __init__(self, func, type_=None):
        self.func = func
        self.type = type_

    def __get__(self, obj, type_=None):
        return self.__class__(self.func.__get__(obj, type_), type_)

    def __call__(self, *args, **kwargs):
        print to_path(self.func, self.type), args, kwargs
        return self.func(*args, **kwargs)

import __builtin__

__builtin__.decorated = decorated
