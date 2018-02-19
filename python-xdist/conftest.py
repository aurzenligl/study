import os
import pytest
from logger import logger, put
import logging
import time
import Pyro4
from queue import Queue, Empty
from decorator import decorator
import inspect

####### pytest_fixture_scope_testrun plugin code #######

def pytest_configure_node(node):
    put('HOOK.pytest_configure_node')
    in_firstnode(node, lambda: setup_daemon(node))
    node.slaveinput['uri'] = node.config.firstnode.slaveinput['uri']

def pytest_unconfigure(config):
    pyro = getattr(config, 'pyro', None)
    if pyro:
        pyro.close()

def in_firstnode(node, fun):
    if getattr(node.config, 'firstnode', None) is None:
        node.config.firstnode = node
    if node.config.firstnode is node:
        fun()

def setup_daemon(node):
    daemon = Pyro4.Daemon()
    uri = daemon.register(TestrunFixtureEngine())

    from threading import Thread
    thr = Thread(target=daemon.requestLoop)
    thr.daemon = True
    thr.start()

    node.config.pyro = daemon
    node.slaveinput['uri'] = str(uri)

@Pyro4.expose
class TestrunFixtureEngine(object):
    def __init__(self):
        self.produce_token = Queue()
        self.produce_token.put(True)
        self.results = Queue()
    def should_produce(self):
        try:
            return self.produce_token.get(False)
        except Empty:
            pass
    def get(self):
        return self.results.get()
    def put(self, x):
        self.results.put(x)

def fixture_scope_testrun(fun):
    if inspect.isgeneratorfunction(fun):
        pytest.fail('testrun fixture must not be a generator')

    def wrap(fun, *args, **kwargs):
        has_session = lambda arg: isinstance(getattr(arg, 'session', None), pytest.Session)
        request = next((arg for arg in args if has_session(arg)), None)
        if not request:
            pytest.fail('testrun fixture must have request argument')

        slaveinput = getattr(request.config, 'slaveinput', None)
        if not slaveinput:
            return fun(*args, **kwargs)

        proxy = Pyro4.Proxy(slaveinput['uri'])
        if proxy.should_produce():
            try:
                ret = fun(*args, **kwargs)
            except Exception as ex:
                ret = ex
            for _ in range(slaveinput['slavecount']):
                try:
                    proxy.put(ret)
                except Exception as ex:
                    proxy.put(ex)
        ret = proxy.get()
        if isinstance(ret, Exception):
            raise ret
        else:
            return ret

    return pytest.fixture(scope='session')(decorator(wrap)(fun))

pytest.fixture_scope_testrun = fixture_scope_testrun

####### pytest_fixture_scope_testrun plugin code end #######

def do_the_long_thing():
    put('LONGTHING.start %.4f' % (time.time()))
    time.sleep(0.5)
    put('LONGTHING.end %.4f' % (time.time()))
    return 42

@pytest.fixture_scope_testrun
def rfix(request):
    put('RFIX.testrun')
    def fin():
        put('RFIX.~testrun')
    request.addfinalizer(fin)
    return do_the_long_thing()

@pytest.fixture(scope='session')
def sfix(rfix):
    put('SFIX.testrun')
    yield
    put('SFIX.~testrun')

@pytest.fixture
def tfix(sfix, rfix):
    put('FIX.rfix=%s' % rfix)
    yield 1

@pytest.mark.tryfirst
def pytest_cmdline_main(config):
    if not getattr(config, 'slaveinput', None):
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
    session = item.session
    slaveinput = getattr(session.config, 'slaveinput', {})
    put('HOOK.pytest_runtest_protocol %s' % slaveinput)
