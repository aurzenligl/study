import time
import pprint
import logging

thelib_logger = logging.getLogger('thelib')
thelib_logger.addHandler(logging.NullHandler())

class logged(object):
    def __init__(self, func, type_=None):
        self.func = func
        self.type = type_

    def __get__(self, obj, type_=None):
        return self.__class__(self.func.__get__(obj, type_), type_)

    def __call__(self, *args, **kwargs):
        indent = ' ' * 4

        def to_str(arg):
            lines = pprint.pformat(arg).splitlines(True)
            return lines[0] + ''.join(indent * 2 + line for line in lines[1:])

        lines = ([_to_path(self.func, self.type)] +
                 [indent + '%s: %s' % (i, to_str(a)) for i, a in enumerate(args)] +
                 [indent + '%s: %s' % (k, to_str(v)) for k, v in kwargs.items()])
        for line in lines:
            thelib_logger.info(line)
        with _Timeit() as t:
            ret = self.func(*args, **kwargs)
        thelib_logger.info(indent + '-> (%.2fs) %s' % (t.duration, ret))
        return ret

def _to_path(fun, cls):
    def cls2str(cls):
        if cls:
            return cls.__name__ + '.'
        else:
            return ''

    physical_path = fun.__module__.replace('.', '/') + '.py:%s' % fun.__code__.co_firstlineno
    logical_path = cls2str(cls) + fun.func_name
    return '%s %s' % (physical_path, logical_path)

class _Timeit:
    def __enter__(self):
        self.start = time.time()
        return self
    def __exit__(self, *_):
        end = time.time()
        self.duration = end - self.start
