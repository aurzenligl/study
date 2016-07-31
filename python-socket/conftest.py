import os
import sys
import pytest

def pytest_addoption(parser):
    parser.addoption ('--count', default=1, type='int', metavar='count',
                      help='Run each test the specified number of times')

def pytest_generate_tests (metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

@pytest.fixture
def user_log_path():
    return 'user_log'

def pytest_configure(config):
    os.system('rm -f user_log')

def pytest_unconfigure(config):
    os.system('cat user_log')
