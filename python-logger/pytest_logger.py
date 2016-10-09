import os
import sys
import re
import pytest
import logging
import time
import datetime
from contextlib import contextmanager

def pytest_addoption(parser):
    group = parser.getgroup("logger", "logging")
    group.addoption('--logdirflat', default=False, action='store_true',
                     help='puts all logs in single file.')

def pytest_configure(config):
    config.pluginmanager.register(LoggerPlugin(config), '_logger')

def pytest_addhooks(pluginmanager):
    pluginmanager.add_hookspecs(LoggerHookspec)

class LoggerPlugin(object):
    def __init__(self, config):
        self.logdirlinks = config.hook.pytest_logger_logdirlink(config=config)
        self.logdirflat = config.getoption('logdirflat')

    def pytest_runtest_setup(self, item):
        def to_loggers(names_list):
            return [logging.getLogger(name) for names in names_list for name in names]

        stdoutloggers = to_loggers(item.config.hook.pytest_logger_stdoutloggers(item=item))
        fileloggers = to_loggers(item.config.hook.pytest_logger_fileloggers(item=item))
        item._logger = state = LoggerState(plugin=self, item=item,
                                           stdoutloggers=stdoutloggers,
                                           fileloggers=fileloggers)
        state.enable()

    def pytest_runtest_teardown(self, item, nextitem):
        handler = item._logger.stdouthandler
        if handler:
            handler.newline_before_next_log()

    def pytest_runtest_makereport(self, item, call):
        if call.when == 'teardown':
            item._logger.disable()

class LoggerState(object):
    FORMAT = '%(asctime)s %(name)s: %(message)s'
    def __init__(self, plugin, item, stdoutloggers, fileloggers):
        self._logsdir = None
        self.plugin = plugin
        self.item = item
        self.stdoutloggers = stdoutloggers
        self.fileloggers = fileloggers
        self.formatter = Formatter(fmt=self.FORMAT)
        self.stdoutloggersandhandlers = stdoutloggers and _make_stdoutloggersandhandlers(self)
        self.fileloggersandhandlers = fileloggers and _make_fileloggersandhandlers(self)
        self.stdouthandler = self.stdoutloggersandhandlers and self.stdoutloggersandhandlers[0][1]
    def logsdir(self):
        self._logsdir = self._logsdir or _make_logsdir(self)
        return self._logsdir
    def logdir(self):
        return self.logsdir().join(_sanitize(self.item.nodeid)).ensure(dir=1)
    def enable(self):
        _add_handlers(self.stdoutloggersandhandlers)
        _add_handlers(self.fileloggersandhandlers)
    def disable(self):
        _remove_handlers(self.fileloggersandhandlers)
        _remove_handlers(self.stdoutloggersandhandlers)

class LoggerHookspec(object):
    def pytest_logger_stdoutloggers(self, item):
        """ called before testcase setup, returns list of logger names """

    def pytest_logger_fileloggers(self, item):
        """ called before testcase setup, returns list of logger names """

    def pytest_logger_logdirlink(self, config):
        """ called after cmdline options parsing, returns location of link to logs dir """

@pytest.fixture
def logdir(request):
    state = request._pyfuncitem._logger
    return state.logdir()

class Formatter(logging.Formatter):
    def __init__(self, *args, **kwargs):
        super(Formatter, self).__init__(*args, **kwargs)
        self._start = time.time()
    def formatTime(self, record, datefmt=None):
        ct = record.created - self._start
        dt = datetime.datetime.utcfromtimestamp(ct)
        return dt.strftime("%M:%S.%f")[:-3]  # omit useconds, leave mseconds

class StdoutHandler(logging.StreamHandler):
    def newline_before_next_log(self):
        if self.stream.name == '<stdout>':
            self.stream.write('\n')

class MyFileHandler(logging.FileHandler):
    def __init__(self, filename, **kwargs):
        logging.FileHandler.__init__(self, filename, **kwargs)

def _sanitize(filename):
    filename = filename.replace('::()::', '-')
    filename = filename.replace('::', '-')
    filename = re.sub(r'\[(.+)\]', r'-\1', filename)
    return filename

def _refresh_link(source, link_name):
    try:
        os.unlink(link_name)
    except OSError:
        pass
    try:
        os.symlink(source, link_name)
    except (OSError, AttributeError, NotImplementedError):
        pass

def _make_logsdir(state):
    logsdir = state.item.config._tmpdirhandler.getbasetemp()
    if logsdir.basename.startswith('popen-gw'):
        logsdir = logsdir.join('..')
    logsdir = logsdir.join('logs').ensure(dir=1)

    for link in state.plugin.logdirlinks:
        _refresh_link(str(logsdir), link)

    return logsdir

def _make_stdout_handler(fmt):
    handler = StdoutHandler(stream=sys.stdout)
    handler.setFormatter(fmt)
    handler.newline_before_next_log()
    return handler

def _make_stdoutloggersandhandlers(state):
    state.stdouthandler = handler = _make_stdout_handler(state.formatter)
    return [(lgr, handler) for lgr in state.stdoutloggers]

def _make_file_handler(logdir, name, fmt):
    logfile = str(logdir.join(name))
    handler = MyFileHandler(filename=logfile, mode='w', delay=True)
    handler.setFormatter(fmt)
    return handler

def _make_fileloggersandhandlers(state):
    logdir = state.logdir()
    if not state.plugin.logdirflat:
        loggers_and_handlers = [
            (lgr, _make_file_handler(logdir, lgr.name, state.formatter))
            for lgr in state.fileloggers
        ]
    else:
        handler = make_handler(logdir, 'logs', state.formatter)
        loggers_and_handlers = [(lgr, handler) for lgr in state.fileloggers]
    return loggers_and_handlers

def _add_handlers(loggers_and_handlers):
    for lgr, hdlr in loggers_and_handlers:
        lgr.addHandler(hdlr)

def _remove_handlers(loggers_and_handlers):
    for lgr, hdlr in loggers_and_handlers:
        lgr.removeHandler(hdlr)
