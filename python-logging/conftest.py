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
loggernames = ['data', 'setup', 'im']

pytest_plugins = 'pytest_logger'

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.warning('before...')
    yield request.config.getoption('error')
    setuplgr.warning('...after')
