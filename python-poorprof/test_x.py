import time
import pytest
from contextlib import contextmanager
from poorprof import poorprof

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
