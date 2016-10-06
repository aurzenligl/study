import os
import sys
import pytest
import logging
import time
from contextlib import contextmanager

'''
cleanup todo list (email todo, written todo, file todo)

1. extract generic logging code to plugin
    - how to configure pytest plugin from conftest?
    - plugin should be enabled automatically
    - adds cmdline options
    - adds fixtures
    - does exactly nothing by default
    - logdir fixture - if used - enables logger plugin operation
    - custom pytest-logger hooks - if used - enable logger plugin operation
        option: --logdirflat [default false]
        option: --logdirlink [provide default in conftest.py]
        hook: pytest_logger_fileloggers [default none, appends]
        hook: pytest_logger_stdoutloggers [default none, appends]
        fixture: logdir
2. add cmdline choice of stdout handlers (default: setup and xystat)
        option: --log [default sut, setup and stat]
3. catch output from subprocess and put via logging to stdout or file
4. fix ~800ms offset of timestamps in test session shorter than 0.1 seconds
5. refactor into a pytest plugin, configured from conftest.py
6. solve "logs" access race condition (simultaneous tests)
'''

class MyStdoutHandler(logging.StreamHandler):
    def __init__(self, *args, **kwargs):
        logging.StreamHandler.__init__(self, *args, **kwargs)
        self._guard = False
    def newline_before_next_log(self):
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
        # TODO: correct subtracting milliseconds
        ct = record.created - self._start
        gt = time.gmtime(ct)
        st = time.strftime("%M:%S", gt)
        t = "%s.%03d" % (st, record.msecs)
        return t

@contextmanager
def handlers_added(loggers_and_handlers):
    for lgr, hdlr in loggers_and_handlers:
        lgr.addHandler(hdlr)
    try:
        yield
    finally:
        for lgr, hdlr in loggers_and_handlers:
            lgr.removeHandler(hdlr)

def refresh_link(source, link_name):
    try:
        os.unlink(link_name)
    except OSError:
        pass
    try:
        os.symlink(source, link_name)
    except (OSError, AttributeError, NotImplementedError):
        pass

class LoggerCfg:
    FORMAT = '%(asctime)s %(name)s: %(message)s'
    def __init__(self, stdoutloggers, fileloggers):
        self.formatter = MyFormatter(fmt=self.FORMAT)
        self.stdouthandler = None
        self.stdoutloggers = stdoutloggers
        self.fileloggers = fileloggers

def pytest_runtest_setup(item):
    # TODO: hooks calls to determine if loggers should be used in this test
    # TODO: hooks should store return values in item._xyz object, which fixture can access
    # TODO: separate file and stdout loggers fixtures

    loggers = [logging.getLogger(name) for name in loggernames]

    item._loggercfg = LoggerCfg(loggers, loggers)
    item.fixturenames.insert(0, '_logger_stdouthandlers')
    item.fixturenames.insert(0, '_logger_filehandlers')

def pytest_runtest_makereport(item, call):
    if call.when == 'call':
        handler = item._loggercfg.stdouthandler
        if handler:
            handler.newline_before_next_log()

@pytest.fixture(scope='session')
def _logsdir(tmpdir_factory):
    logsdir = tmpdir_factory.getbasetemp()
    if logsdir.basename.startswith('popen-gw'):
        logsdir = logsdir.join('..')
    logsdir = logsdir.join('logs').ensure(dir=1)
    refresh_link(str(logsdir), os.path.join(linkdir, 'logs'))
    return logsdir

@pytest.fixture
def _logdir(_logsdir, request):
    def sanitize(filename):
        import string
        tbl = string.maketrans('[]:', '___')
        return filename.translate(tbl)

    return _logsdir.join(sanitize(request.node.nodeid)).ensure(dir=1)

@pytest.yield_fixture
def _logger_stdouthandlers(request):
    def make_handler(fmt):
        handler = MyStdoutHandler(stream=sys.stdout)
        handler.setFormatter(fmt)
        handler.newline_before_next_log()
        return handler

    cfg = request._pyfuncitem._loggercfg
    cfg.stdouthandler = handler = make_handler(cfg.formatter)
    loggers_and_handlers = [(lgr, handler) for lgr in cfg.stdoutloggers]

    with handlers_added(loggers_and_handlers):
        yield

@pytest.yield_fixture
def _logger_filehandlers(_logdir, request):
    def make_handler(logdir, fmt, logger):
        logfile = str(logdir.join(logger.name))
        handler = MyFileHandler(filename=logfile, mode='w', delay=True)
        handler.setFormatter(fmt)
        return handler

    cfg = request._pyfuncitem._loggercfg
    loggers_and_handlers = [
        (lgr, make_handler(_logdir, cfg.formatter, lgr))
        for lgr in cfg.fileloggers
    ]

    with handlers_added(loggers_and_handlers):
        yield

@pytest.fixture
def logdir(_logdir):
    return _logdir

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

linkdir = os.path.dirname(__file__)
loggers = [logging.getLogger(name) for name in ('data', 'setup', 'im')]
loggernames = ['data', 'setup', 'im']

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
