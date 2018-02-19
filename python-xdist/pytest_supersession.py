import pytest
import inspect
from queue import Queue, Empty
from threading import Thread
import Pyro4
from decorator import decorator

def pytest_configure_node(node):
    if 'pyro' not in dir(node.config):
        setup_pyro(node.config)
    node.slaveinput['uri'] = node.config.pyro_uri

def pytest_unconfigure(config):
    pyro = getattr(config, 'pyro', None)
    if pyro:
        pyro.close()

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

def setup_pyro(config):
    daemon = Pyro4.Daemon()
    uri = daemon.register(TestrunFixtureEngine())

    thr = Thread(target=daemon.requestLoop)
    thr.daemon = True
    thr.start()

    config.pyro = daemon
    config.pyro_uri = str(uri)

def supersession_fixture(orig, *args):
    if not getattr(supersession_fixture, 'used', None):
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
            if proxy.should_produce():
                try:
                    ret = fun(*args)
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
