import time
import pytest
from contextlib import contextmanager

class state:
    stack = []
    end = None
    @staticmethod
    def indent(mod = 0):
        return (len(state.stack) + mod) * 4 * ' '

def next_suf(suf):
    return chr(ord(suf) + 1)

class poorprof(object):
    def __init__(self,name):
        self.name = name
        self.suffix = None
    def __enter__(self):
        if state.stack:
            start = state.stack[-1]._intermittent()
        else:
            start = time.time()
        if not state.stack and state.end:
            print('pp: %6s %6s %s' % ('%.3f' % (start-state.end), '', '--'))
            state.end = None
        self.rstart = self.start = start
        state.stack.append(self)
    def __exit__(self, ty, val, tb):
        assert state.stack.pop() is self
        if state.stack:
            state.stack[-1]._reset()
        end = time.time()
        if not state.stack:
            state.end = end
        self.suffix = next_suf(self.suffix) if self.suffix else self.suffix
        if self.suffix:
            whole = '%.3f' % (end-self.rstart)
        else:
            whole = ''
        print('pp: %6s %6s %s%s' % ('%.3f' % (end-self.start), whole, state.indent(), repr(self)))
        return False
    def _intermittent(self):
        self.suffix = next_suf(self.suffix) if self.suffix else 'a'
        start, newstart = self._reset()
        print('pp: %6s %6s %s%s' % ('%.3f' % (newstart-start), '', state.indent(-1), repr(self)))
        return newstart
    def _reset(self):
        start = self.start
        self.start = time.time()
        return start, self.start
    def __repr__(self):
        return '%s.%s' % (self.name, self.suffix) if self.suffix else self.name

'''
0000.00         context.a
0000.00             bar_start
0000.00             foo_start
0000.00         context.b
0000.00             fixtures.a
0000.00                 testcase
0000.00 0000.00     fixtures.b
0000.00             foo_stop
0000.00 0000.00 context.c
'''

def bar():
    time.sleep(0.1)
    return 18

@contextmanager
def foo():
    with poorprof("foo_start"):
        time.sleep(0.1)
    yield 42
    with poorprof("foo_stop"):
        time.sleep(0.1)

@pytest.fixture
def context():
    time.sleep(0.05)
    with poorprof("context"):
        time.sleep(0.03)
        with poorprof("bar_start"):
            bar()
        with foo():
            with poorprof("fixtures"):
                yield

@pytest.fixture
def fix(context):
    time.sleep(0.1)
    yield
    time.sleep(0.1)

def test_x(fix):
    with poorprof("testcase"):
        time.sleep(0.06)
