import os
import pytest
import logging
from process import process

'''
cleanup todo list (email todo, written todo, file todo)

multiple hooks
nested conftests
logging fixtures
logging libraries
log levels
incorrect logger names
parametrized tests
tests as class methods
log from subprocess
log format override

1. see other plugins
2. see cookie cutter
3. see plugin tests
4. plugin repo + layout
5. test plugin
6. readme to explain API
7. docstrings for functions, classes
8. release on pypi
'''

must_loggers = ['problem']
configurable_loggers = ['setup', 'im', 'data', 'proc', 'daemon']
default_loggers = ['setup']

def pytest_addoption(parser):
    def comma_delimited_loggers(delimited_strings):
        loggernames = delimited_strings.split(',')
        for name in loggernames:
            if name not in configurable_loggers:
                import argparse
                raise argparse.ArgumentTypeError('logger "%s" not found, choose from: %s'
                                                 % (name, ', '.join(configurable_loggers)))
        return loggernames

    parser.addoption('--count', type='int', metavar='COUNT',
                     help='Run each test the specified number of times')
    parser.addoption('--error', action='store_true',
                     help='Cause failure during test run')
    parser.addoption('--log', default=default_loggers, metavar='LOGGER,...',
                     type=comma_delimited_loggers,
                     help='Pick configurable loggers for stdout from: %s'
                        % ', '.join(configurable_loggers))
    parser.addoption('--logall', action='store_true',
                     help='Pick all loggers for stdout')
    parser.addoption('--logflat', action='store_true',
                     help='Log all to single logs file.')

def pytest_logger_stdoutloggers(item):
    option = item.config.option
    return must_loggers + (option.logall and configurable_loggers or option.log)

def pytest_logger_fileloggers(item):
    logflat = item.config.option.logflat
    return logflat and [''] or (must_loggers + configurable_loggers)

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_generate_tests(metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

daemonlgr = logging.getLogger('daemon')
daemonlgr.addHandler(logging.NullHandler())

logging.root.setLevel(logging.NOTSET)

@pytest.yield_fixture(scope='session', autouse=True)
def sessionfixture():
    setuplgr.info('session begins')
    yield
    setuplgr.info('session ends')

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.info('before...')
    yield request.config.getoption('error')
    setuplgr.info('...after')

@pytest.yield_fixture(scope='session')
def daemon(request):
    with process('./printdaemon', stdoutlgr=daemonlgr):
        yield
