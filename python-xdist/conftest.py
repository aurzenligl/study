import os
import pytest
from logger import put
import time

pytest.register_assert_rewrite('pytest_supersession')
__import__('pytest_supersession')
pytest_plugins = 'pytest_supersession'

def do_the_long_thing():
    put('LONGTHING.start')
    time.sleep(0.5)
    put('LONGTHING.end')
    return 42

@pytest.fixture(scope='supersession')
def rfix(request):
    put('RFIX.testrun')
    def fin():
        put('RFIX.~testrun')
    request.addfinalizer(fin)
    return do_the_long_thing()

@pytest.fixture(scope='session')
def sfix(rfix, request):
    put('SFIX.testrun')
    yield
    put('SFIX.~testrun')

@pytest.fixture
def tfix(sfix, rfix):
    put('FIX.rfix=%s' % rfix)
    yield 1

@pytest.mark.tryfirst
def pytest_cmdline_main(config):
    if 'slaveinput' in dir(config):
        return

    if config.getvalue('poolmode'):
        config.option.numprocesses = 3

def pytest_addoption(parser):
    parser.addoption('--poolmode', action='store_true',
                     help='Dummy variable just for the heck of it')

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_logger_config(logger_config):
    logger_config.add_loggers(['setup'], stdout_level='info')
    logger_config.set_log_option_default('setup')

def pytest_runtest_protocol(item, nextitem):
    if 'testinit' in dir(item.config):
        return
    item.config.testinit = True

    session = item.session
    slaveinput = getattr(session.config, 'slaveinput', {})
    put('HOOK.pytest_runtest_protocol %s' % slaveinput)
