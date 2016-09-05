import os
import sys
import pytest
import logging
import time
from distutils.log import INFO
from reportlab.rl_settings import odbc_driver

'''
1. add file handlers, which log to logs directory, one file per logger
2. add cmdline choice of stdout handlers (default: setup and xystat)
3. catch output from subprocess and put via logging to stdout or file
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

class NewlineGuard(object):
    def __init__(self):
        self._write_newline_once = False
    def write_newline_once(self):
        self._write_newline_once = True
    def check(self):
        if self._write_newline_once:
            self._write_newline_once = False
            return True
        return False

class MyHandler(logging.StreamHandler):
    def __init__(self, *args, **kwargs):
        super(MyHandler, self).__init__(*args, **kwargs)
        self._guard = None
    def set_guard(self, guard):
        self._guard = guard
    def write_initial_newline(self):
        MyHandler._write_initial_newline = True
    def emit(self, record):
        if self._guard and self._guard.check():
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
    info = []
    guard = NewlineGuard()

    FORMAT = '%(asctime)s %(name)s: %(message)s'
    fmt = MyFormatter(fmt=FORMAT)
    for name in loggers:
        hdlr = MyHandler(stream=sys.stdout)
        hdlr.set_guard(guard)
        hdlr.setFormatter(fmt)
        lgr = logging.getLogger(name)
        lgr.addHandler(hdlr)
        info.append((lgr, hdlr))

    return info, guard

def remove_loggers(info):
    for lgr, hdlr in info:
        lgr.removeHandler(hdlr)

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

def pytest_runtest_setup(item):
    info, guard = setup_loggers()
    guard.write_newline_once()
    item.loginfo = info
    item.logguard = guard

def pytest_runtest_makereport(item, call):
    if call.when == 'call':
        item.logguard.write_newline_once()
    elif call.when == 'teardown':
        remove_loggers(item.loginfo)

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
