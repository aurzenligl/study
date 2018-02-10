import time
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
        lines = ([_to_path(self.func, self.type)] +
                 ['%sarg[%s]=%s' % (indent, i, a) for i, a in enumerate(args)] +
                 ['%s%s=%s' % (indent, k, v) for k, v in kwargs.items()])
        for line in lines:
            thelib_logger.info(line)
        with timeit() as t:
            ret = self.func(*args, **kwargs)
        thelib_logger.info('-> (%.2fs) %s' % (t.duration, ret))
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

class timeit:
    def __enter__(self):
        self.start = time.time()
        return self
    def __exit__(self, *_):
        end = time.time()
        self.duration = end - self.start
