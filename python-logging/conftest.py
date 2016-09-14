import os
import sys
import pytest
import logging
import time

'''
cleanup todo list (email todo, written todo, file todo)

1. add file handlers, which log to logs directory, one file per logger
2. add cmdline choice of stdout handlers (default: setup and xystat)
3. catch output from subprocess and put via logging to stdout or file
4. fix ~800ms offset of timestamps in test session shorter than 0.1 seconds
5. refactor into a pytest plugin, configured from conftest.py
6. solve "logs" access race condition (simultaneous tests)
'''

def pytest_addoption(parser):
    parser.addoption ('--count', default=1, type='int', metavar='count',
                      help='Run each test the specified number of times')
    parser.addoption ('--error', action='store_true',
                      help='Cause failure during test run')

def pytest_generate_tests(metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

class MyStdoutHandler(logging.StreamHandler):
    def __init__(self, *args, **kwargs):
        logging.StreamHandler.__init__(self, *args, **kwargs)
        self._guard = False
    def write_initial_newline(self):
        self._guard = True
    def emit(self, record):
        if self._guard:
            self._guard = False
            if self.stream.name == '<stdout>':
                self.stream.write('\n')
        logging.StreamHandler.emit(self, record)

class MyFileHandler(logging.FileHandler):
    def __init__(self, filename, **kwargs):
        logging.FileHandler.__init__(self, filename, **kwargs)
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

def ensure_dir(path):
    try:
        os.makedirs(path)
    except OSError as e:
        import errno
        if e.errno != errno.EEXIST:
            raise

loggers = [logging.getLogger(name) for name in ('data', 'setup', 'im')]

def sanitize(filename):
    import string
    tbl = string.maketrans('[]:', '___')
    return filename.translate(tbl)

def make_file_handler(logfile, fmt):
    ensure_dir(os.path.dirname(logfile))
    hfile = MyFileHandler(filename=logfile, mode='w', delay=True)
    hfile.setFormatter(fmt)
    return hfile

def setup_loggers(item):
    FORMAT = '%(asctime)s %(name)s: %(message)s'
    fmt = MyFormatter(fmt=FORMAT)

    hstdout = MyStdoutHandler(stream=sys.stdout)
    hstdout.setFormatter(fmt)

    handlers = []
    for lgr in loggers:
        hfilename = os.path.join('logs', sanitize(item.nodeid), lgr.name)
        hfile = make_file_handler(hfilename, fmt)
        handlers.append((lgr, hfile))
        handlers.append((lgr, hstdout))

    for lgr, hdlr in handlers:
        lgr.addHandler(hdlr)

    def finalize():
        for lgr, hdlr in handlers:
            lgr.removeHandler(hdlr)

    return hstdout, finalize

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

def pytest_runtest_setup(item):
    item.logguard, item.logfinalizer = setup_loggers(item)
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
