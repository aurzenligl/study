import os
import pytest
from logger import logger, put
import logging

@pytest.fixture(scope='session')
def sfix(request):
    put('FIX.pytest.fixture.session')
    logger.info('sfix')
    yield 1
    logger.info('~sfix')

@pytest.fixture
def tfix(sfix):
    put('FIX.pytest.fixture')
    logger.info('tfix')
    yield 1
    logger.info('~tfix')

def pytest_addoption(parser):
    put('HOOK.pytest_addoption')
    parser.addoption('--dummy', type='int', metavar='COUNT',
                     help='Dummy variable just for the heck of it')

def pytest_logger_logdirlink(config):
    put('HOOK.pytest_logger_logdirlink')
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_logger_config(logger_config):
    put('HOOK.pytest_logger_config')
    logger_config.add_loggers(['setup'], stdout_level='info')
    logger_config.set_log_option_default('setup')



def pytest_cmdline_preparse(config, args):
    put('HOOK.pytest_cmdline_preparse')

def pytest_generate_tests(metafunc):
    put('HOOK.pytest_generate_tests')

def pytest_runtestloop(session):
    put('HOOK.pytest_runtestloop')

def pytest_runtestloop(session):
    put('HOOK.pytest_runtestloop')

def pytest_runtest_protocol(item, nextitem):
    put('HOOK.pytest_runtest_protocol')

def pytest_collection_modifyitems(session, config, items):
    put('HOOK.pytest_collection_modifyitems')
