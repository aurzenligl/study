import time
import pprint
import logging
from functools import wraps
from inspect import isclass

logger = logging.getLogger('thelib')
logger.addHandler(logging.NullHandler())

def logged(fun):
    desc = next((desc for desc in (staticmethod, classmethod) if isinstance(fun, desc)), None)
    if desc:
        fun = fun.__func__

    @wraps(fun)
    def wrap(*args, **kwargs):
        cls, nonselfargs = declassify(fun, args)

        if _logging.currently:
            return fun(*args, **kwargs)

        with _logging:
            for line in func_call_to_lines(fun, cls, nonselfargs, kwargs):
                output(line)
            with Timed() as t:
                ret = fun(*args, **kwargs)
            for line in func_ret_to_lines(t.duration, ret):
                output(line)
            return ret

    wrap.wrapped = fun

    if desc:
        wrap = desc(wrap)
    return wrap

def declassify(fun, args):
    if len(args):
        met = getattr(args[0], fun.__name__, None)
        if met:
            wrap = getattr(met, '__func__', None)
            if wrap:
                wrapped = getattr(wrap, 'wrapped', None)
                if wrapped is fun:
                    maybe_cls = args[0]
                    cls = maybe_cls if isclass(maybe_cls) else maybe_cls.__class__
                    return cls, args[1:]
    return None, args

def output(line):
    logger.info(line)
    write_to_file(line)

def func_call_to_lines(fun, cls, args, kwargs):
    return [to_path(fun, cls)] + indent(flatten_lines(
        ['%s: %s' % (i, pprint.pformat(a)) for i, a in enumerate(args)] +
        ['%s: %s' % (k, pprint.pformat(v)) for k, v in kwargs.items()]
    ))

def func_ret_to_lines(duration, ret):
    return indent(flatten_lines(['-> (%.2fs) %s' % (duration, pprint.pformat(ret))]))

def flatten_lines(lines):
    out = []
    for line in lines:
        rawlines = line.splitlines()
        out.append(rawlines[0])
        out.extend(indent(rawlines[1:]))
    return out

def to_path(fun, cls):
    physical_path = fun.__module__.replace('.', '/') + '.py:%s' % fun.__code__.co_firstlineno
    logical_path = (cls.__name__ + '.' if cls else '') + fun.func_name
    return '%s %s' % (physical_path, logical_path)

def indent(lines):
    indent = ' ' * 4
    return [indent + line for line in lines]

class Timed:
    def __enter__(self):
        self.start = time.time()
        return self

    def __exit__(self, *_):
        end = time.time()
        self.duration = end - self.start

class Logging:
    def __init__(self):
        self.currently = False

    def __enter__(self):
        self.currently = True

    def __exit__(self, *_):
        self.currently = False

_logging = Logging()

def maketmp():
    import os
    try:
        os.mkdir('/tmp/luger')
    except OSError:
        pass

def write_to_file(line):
    import os
    with open('/tmp/luger/%s' % os.getpid(), 'a') as f:
        relnow = time.time() - _import_time
        f.write('{0:08.3f}: {1}\n'.format(relnow, line))

def output_cmdline():
    import sys
    output('cmdline args: %s' % sys.argv)

_import_time = time.time()
maketmp()
output_cmdline()
