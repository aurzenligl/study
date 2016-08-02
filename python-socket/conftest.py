import os
import sys
import pytest
import persistent

def pytest_addoption(parser):
    parser.addoption ('--count', default=1, type='int', metavar='count',
                      help='Run each test the specified number of times')
    parser.addoption ('--sockets', default=1, type='int', metavar='sockets',
                      help='Run each test the specified number of times')

def pytest_generate_tests (metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

@pytest.fixture
def user_log_path():
    return 'user_log'

@pytest.fixture
def socket_count(pytestconfig):
    return pytestconfig.option.sockets

@pytest.fixture(scope='session')
def portalloc():
    return persistent.PortAllocator('/tmp/pytest-sockets', 20000)

def pytest_configure(config):
    os.system('rm -f user_log')
