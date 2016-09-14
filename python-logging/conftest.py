import os
import sys
import pytest
import logging
import time

'''
1. add file handlers, which log to logs directory, one file per logger
2. add cmdline choice of stdout handlers (default: setup and xystat)
3. catch output from subprocess and put via logging to stdout or file
4. fix ~800ms offset of timestamps in test session shorter than 0.1 seconds
5. refactor into a pytest plugin, configured from conftest.py
'''

def pytest_addoption(parser):
    parser.addoption ('--count', default=1, type='int', metavar='count',
                      help='Run each test the specified number of times')
    parser.addoption ('--error', action='store_true',
                      help='Cause failure during test run')

def pytest_generate_tests (metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

class MyHandler(logging.StreamHandler):
    def __init__(self, *args, **kwargs):
        super(MyHandler, self).__init__(*args, **kwargs)
        self._guard = False
    def write_initial_newline(self):
        self._guard = True
    def emit(self, record):
        if self._guard:
            self._guard = False
            if self.stream.name == '<stdout>':
                self.stream.write('\n')
        super(MyHandler, self).emit(record)

class MyFormatter(logging.Formatter):
    def __init__(self, *args, **kwargs):
        super(MyFormatter, self).__init__(*args, **kwargs)
        self._start = time.time()
    def formatTime(self, record, datefmt=None):
        ct = record.created - self._start
        gt = time.gmtime(ct)
        st = time.strftime("%M:%S", gt)
        t = "%s.%03d" % (st, record.msecs)
        return t

loggers = ['data', 'setup', 'im']

def setup_loggers():
    FORMAT = '%(asctime)s %(name)s: %(message)s'
    fmt = MyFormatter(fmt=FORMAT)
    hdlr = MyHandler(stream=sys.stdout)
    hdlr.setFormatter(fmt)

    for name in loggers:
        lgr = logging.getLogger(name)
        lgr.addHandler(hdlr)

    def finalize():
        for name in loggers:
            lgr = logging.getLogger(name)
            lgr.removeHandler(hdlr)

    return hdlr, finalize

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

def pytest_runtest_setup(item):
    item.logguard, item.logfinalizer = setup_loggers()
    item.logguard.write_initial_newline()

def pytest_runtest_makereport(item, call):
    if call.when == 'call':
        item.logguard.write_initial_newline()
    elif call.when == 'teardown':
        item.logfinalizer()

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
