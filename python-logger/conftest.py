import os
import sys
import pytest
import logging
from process import process

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

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_logger_config(logger_config):
    logger_config.add_loggers(['setup', 'im', 'data', 'proc', 'daemon'], stdout_level=logging.INFO)
    logger_config.set_log_option_default('setup,im')

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
