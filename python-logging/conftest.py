import os
import sys
import pytest
import logging
import time

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
        self._write_initial_newline = False
    def write_initial_newline(self):
        self._write_initial_newline = True
    def emit(self, record):
        if self._write_initial_newline:
            self._write_initial_newline = False
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

loggers = ['lgr', 'setup', 'im']

def setup_loggers():
    def logmake(name, hdlr):
        lgr = logging.getLogger(name)
        lgr.addHandler(hdlr)
        return lgr

    FORMAT = '%(asctime)s %(name)s: %(message)s'
    fmt = MyFormatter(fmt=FORMAT)
    hdlr = MyHandler(stream = sys.stdout)
    hdlr.setFormatter(fmt)

    for lgr in loggers:
        logmake(lgr, hdlr)

    return hdlr

def remove_loggers(hdlr):
    for name in loggers:
        lgr = logging.getLogger(name)
        lgr.removeHandler(hdlr)

def try_to_newline(hdlr):
    if not hasattr(hdlr, 'stream'):
        return
    if hdlr.stream.name == '<stdout>':
        hdlr.write_initial_newline()

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

def pytest_runtest_setup(item):
    item.log_handler = setup_loggers()
    try_to_newline(item.log_handler)

def pytest_runtest_makereport(item, call):
    if call.when == 'call':
        try_to_newline(item.log_handler)
    elif call.when == 'teardown':
        remove_loggers(item.log_handler)

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
