import os
import pytest
from logger import logger, put
import logging
import time
import Pyro4
from queue import Queue, Empty
from decorator import decorator
import inspect

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

def fixture_scope_testrun(*args, **kwargs):
    if not getattr(fixture_scope_testrun, 'used', None):
        fixture_scope_testrun.used = True
    else:
        pytest.fail('only one supersession fixture supported')

    def the_decorator(fun):
        if inspect.isgeneratorfunction(fun):
            pytest.fail('supersession fixture must not be a generator')

        def wrap(fun, *args, **kwargs):
            has_session = lambda arg: isinstance(getattr(arg, 'session', None), pytest.Session)
            request = next((arg for arg in args if has_session(arg)), None)
            if not request:
                pytest.fail('supersession fixture must have request argument')

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

        kwargs['scope'] = 'session'
        return pytest.fixture(*args, **kwargs)(decorator(wrap)(fun))
    return the_decorator

_pytest_fixture = pytest.fixture

def fixture(*args, **kwargs):
    scope = kwargs.get('scope')
    if scope and scope == 'supersession':
        return fixture_scope_testrun(*args, **kwargs)
    else:
        global _pytest_fixture
        return _pytest_fixture(*args, **kwargs)

pytest.fixture = fixture
