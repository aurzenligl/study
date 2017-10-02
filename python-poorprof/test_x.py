import time
import pytest
from contextlib import contextmanager

class state:
    stack = []
    end = None
    @staticmethod
    def indent():
        return (len(state.stack) - 1) * 4 * ' '

def next_suf(suf):
    return chr(ord(suf) + 1)

def report(name, part, whole=None):
    spart = '%.3f' % part if part else ''
    swhole = '%.3f' % whole if whole else ''
    print('pp: %6s %6s %s%s' % (spart, swhole, state.indent(), name))

class poorprof(object):
    def __init__(self,name):
        self.name = name
        self.count = 0

    def _report(self, exit=False):
        end = time.time()
        part = '%.3f' % (end - self.start)
        whole = '%.3f' % (end - self.rstart) if exit and self.count else ''
        name = self.name if exit and not self.count else self.name + '.' + chr(ord('a') + self.count)
        self.count += 1
        print('pp: %6s %6s %s%s' % (part, whole, state.indent(), name))
        return end

    def __enter__(self):
        if state.stack:
            start = state.stack[-1]._intermittent()
        else:
            start = time.time()
        self.rstart = self.start = start
        state.stack.append(self)

        if state.end:
            report('--', start-state.end)
            state.end = None

    def __exit__(self, ty, val, tb):
        end = self._report(exit=True)
        assert state.stack.pop() is self
        if state.stack:
            state.stack[-1].start = end
        else:
            state.end = end
        return False

    def _intermittent(self):
        self.start = self._report()
        return self.start
        # TODO return self.start

    def _reset(self):
        start = self.start
        self.start = time.time()
        return start, self.start

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
