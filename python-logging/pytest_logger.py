import os
import sys
import time
import pytest
import logging
from contextlib import contextmanager

def pytest_configure(config):
    config.pluginmanager.register(LoggerPlugin(config), '_logger')

def pytest_addhooks(pluginmanager):
    pluginmanager.add_hookspecs(LoggerHookspec)

class LoggerPlugin(object):
    def __init__(self, config):
        self.logdirlinks = config.hook.pytest_logger_logdirlink(config=config)

    def pytest_runtest_setup(self, item):
        def to_loggers(names_list):
            return [logging.getLogger(name) for names in names_list for name in names]

        stdoutloggers = to_loggers(item.config.hook.pytest_logger_stdoutloggers(item=item))
        fileloggers = to_loggers(item.config.hook.pytest_logger_fileloggers(item=item))

        item._logger = LoggerState(plugin=self, stdoutloggers=stdoutloggers, fileloggers=fileloggers)

        if fileloggers:
            item.fixturenames.insert(0, '_filehandlers')
        if stdoutloggers:
            item.fixturenames.insert(0, '_stdouthandlers')

    def pytest_runtest_makereport(self, item, call):
        if call.when == 'call':
            handler = item._logger.stdouthandler
            if handler:
                handler.newline_before_next_log()

class LoggerState(object):
    FORMAT = '%(asctime)s %(name)s: %(message)s'
    def __init__(self, plugin, stdoutloggers, fileloggers):
        self.plugin = plugin
        self.stdoutloggers = stdoutloggers
        self.fileloggers = fileloggers
        self.formatter = Formatter(fmt=self.FORMAT)
        self.stdouthandler = None

class LoggerHookspec(object):
    def pytest_logger_stdoutloggers(self, item):
        """ called before testcase setup, returns list of logger names """

    def pytest_logger_fileloggers(self, item):
        """ called before testcase setup, returns list of logger names """

    def pytest_logger_logdirlink(self, config):
        """ called after cmdline options parsing, returns location of link to logs dir """

@pytest.fixture(scope='session')
def _logsdir(tmpdir_factory, request):
    logsdir = tmpdir_factory.getbasetemp()
    if logsdir.basename.startswith('popen-gw'):
        logsdir = logsdir.join('..')
    logsdir = logsdir.join('logs').ensure(dir=1)

    state = request._pyfuncitem._logger
    for link in state.plugin.logdirlinks:
        _refresh_link(str(logsdir), link)

    return logsdir

@pytest.fixture
def _logdir(_logsdir, request):
    def sanitize(filename):
        import string
        tbl = string.maketrans('[]:', '___')
        return filename.translate(tbl)

    return _logsdir.join(sanitize(request.node.nodeid)).ensure(dir=1)

@pytest.yield_fixture
def _stdouthandlers(request):
    def make_handler(fmt):
        handler = StdoutHandler(stream=sys.stdout)
        handler.setFormatter(fmt)
        handler.newline_before_next_log()
        return handler

    state = request._pyfuncitem._logger
    state.stdouthandler = handler = make_handler(state.formatter)
    loggers_and_handlers = [(lgr, handler) for lgr in state.stdoutloggers]

    with _handlers_added(loggers_and_handlers):
        yield

@pytest.yield_fixture
def _filehandlers(_logdir, request):
    def make_handler(logdir, fmt, logger):
        logfile = str(logdir.join(logger.name))
        handler = MyFileHandler(filename=logfile, mode='w', delay=True)
        handler.setFormatter(fmt)
        return handler

    state = request._pyfuncitem._logger
    loggers_and_handlers = [
        (lgr, make_handler(_logdir, state.formatter, lgr))
        for lgr in state.fileloggers
    ]

    with _handlers_added(loggers_and_handlers):
        yield

logdir = _logdir

class Formatter(logging.Formatter):
    def __init__(self, *args, **kwargs):
        super(Formatter, self).__init__(*args, **kwargs)
        self._start = time.time()
    def formatTime(self, record, datefmt=None):
        # TODO: correct subtracting milliseconds
        ct = record.created - self._start
        gt = time.gmtime(ct)
        st = time.strftime("%M:%S", gt)
        t = "%s.%03d" % (st, record.msecs)
        return t

class StdoutHandler(logging.StreamHandler):
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

@contextmanager
def _handlers_added(loggers_and_handlers):
    for lgr, hdlr in loggers_and_handlers:
        lgr.addHandler(hdlr)
    try:
        yield
    finally:
        for lgr, hdlr in loggers_and_handlers:
            lgr.removeHandler(hdlr)

def _refresh_link(source, link_name):
    try:
        os.unlink(link_name)
    except OSError:
        pass
    try:
        os.symlink(source, link_name)
    except (OSError, AttributeError, NotImplementedError):
        pass
