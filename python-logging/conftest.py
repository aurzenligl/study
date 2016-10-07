import os
import pytest
import logging

'''
1. extract generic logging code to plugin
    - adds cmdline options
        option: --logdirflat [default false]
2. conftest.py: adds cmdline choice of stdout handlers (default: setup and xystat)
        option: --log [default sut, setup and stat]
3. catch output from subprocess and put via logging to stdout or file
4. fix ~800ms offset of timestamps in test session shorter than 0.1 seconds
5. sanitize in a saner way

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

def pytest_logger_stdoutloggers(item):
    return ['data', 'setup', 'im']

def pytest_logger_fileloggers(item):
    return ['data', 'setup', 'im']

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

pytest_plugins = 'pytest_logger'

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
