import inspect
import pytest
import Pyro4
from itertools import count
from queue import Queue, Empty
from threading import Thread, Event
from decorator import decorator

def pytest_configure_node(node):
    if 'pyro' not in dir(node.config):
        setup_pyro(node.config)
        node.config.slave_count = count(1)
    node.slaveinput['uri'] = node.config.pyro_uri
    node.slaveinput['index'] = next(node.config.slave_count)

def pytest_unconfigure(config):
    pyro = getattr(config, 'pyro', None)
    if pyro:
        pyro.close()

@Pyro4.expose
class SuperSessionMediator(object):
    def __init__(self):
        self.request = Queue()
        self.request.put(True)
        self.ready = Event()
        self.result = None
        self.exc = None

    def request_production(self):
        try:
            return self.request.get(block=False)
        except Empty:
            return False

    def set(self, res):
        self.result = res
        self.ready.set()

    def set_exception(self, exc):
        self.exc = exc
        self.ready.set()

    def get(self):
        self.ready.wait()
        if self.exc:
            raise self.exc
        else:
            return self.result

def setup_pyro(config):
    daemon = Pyro4.Daemon()
    uri = daemon.register(SuperSessionMediator())

    thr = Thread(target=daemon.requestLoop)
    thr.daemon = True
    thr.start()

    config.pyro = daemon
    config.pyro_uri = str(uri)

def supersession_fixture(orig, *args):
    if 'used' not in dir(supersession_fixture):
        supersession_fixture.used = True
    else:
        pytest.fail('only one supersession fixture supported')

    def the_decorator(fun):
        if inspect.isgeneratorfunction(fun):
            pytest.fail('supersession fixture must not be a generator')

        def wrap(fun, *args):
            has_session = lambda arg: isinstance(getattr(arg, 'session', None), pytest.Session)
            request = next((arg for arg in args if has_session(arg)), None)
            if not request:
                pytest.fail('supersession fixture must have request argument')

            slaveinput = getattr(request.config, 'slaveinput', None)
            if not slaveinput:
                return fun(*args)

            proxy = Pyro4.Proxy(slaveinput['uri'])
            if proxy.request_production():
                try:
                    res = fun(*args)
                    proxy.set(res)
                    return res
                except Exception as ex:
                    proxy.set_exception(str(ex))
                    raise
            try:
                return proxy.get()
            except Exception as ex:
                pytest.fail('Remote exception: %s' % ex)

        return orig(*args)(decorator(wrap, fun))
    return the_decorator

def fixture(orig, *args):
    scope = args[0]
    if scope == 'supersession':
        args = ('session',) + args[1:]
        return supersession_fixture(orig, *args)
    else:
        return orig(*args)

pytest.fixture = decorator(fixture, pytest.fixture)
