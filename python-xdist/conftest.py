import os
import pytest
from logger import logger, put
import logging
import time
import Pyro4
from queue import Queue, Empty

def do_the_long_thing():
    put('LONGTHING.start %.4f' % (time.time()))
    time.sleep(0.5)
    put('LONGTHING.end %.4f' % (time.time()))
    return 42

@pytest.fixture(scope='session')
def sfix(request):
    uri = request.config.slaveinput['uri']
    gm = Pyro4.Proxy(uri)
    if gm.should_produce():
        status = do_the_long_thing()
        for _ in range(request.config.slaveinput['slavecount']):
            gm.put(status)
    res = gm.get()

    put('FIX.pytest.fixture.session %s' % res)
    logger.info('sfix')
    yield 1
    logger.info('~sfix')

@pytest.fixture
def tfix(sfix):
    #put('FIX.pytest.fixture')
    logger.info('tfix')
    yield 1
    logger.info('~tfix')

def pytest_addoption(parser):
    #put('HOOK.pytest_addoption')
    parser.addoption('--dummy', type='int', metavar='COUNT',
                     help='Dummy variable just for the heck of it')

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_logger_config(logger_config):
    logger_config.add_loggers(['setup'], stdout_level='info')
    logger_config.set_log_option_default('setup')

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

def pytest_configure_node(node):
    put('HOOK.pytest_configure_node')
    in_firstnode(node, lambda: setup_daemon(node))
    node.slaveinput['uri'] = node.config.firstnode.slaveinput['uri']

def pytest_testnodedown(node, error):
    in_firstnode(node, lambda: node.pyro.close())
    put('HOOK.pytest_testnodedown')

def pytest_runtest_protocol(item, nextitem):
    session = item.session
    put('HOOK.pytest_runtest_protocol %s' % session.config.slaveinput)

#######

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

    node.pyro = daemon
    node.slaveinput['uri'] = str(uri)
