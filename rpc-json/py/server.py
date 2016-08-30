#!/usr/bin/env python

from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
from bunch import Bunch
from mock import Mock
from pprint import pformat
import types

def _get_public_methods(instance):
    ret = []
    for name in type(instance).__dict__.keys():
        if not name.startswith("_"):
            obj = getattr(instance, name)
            if isinstance(obj, types.MethodType):
                ret.append(obj)
    return ret

class Mocker(object):
    def __init__(self):
        self.mocks = Bunch()

    def register_function(self, function):
        mock_ = self.mocks.get(function.__name__, None)
        assert mock_ is None or mock_.size_effect == self._noop_func
        self.mocks[function.__name__] = Mock(side_effect=function)

    def unregister_function(self, function):
        mock_ = self.mocks.get(function.__name__, None)
        assert mock_ is not None
        mock_.configure_mock(_mock_wraps=self._noop_func)

    def register_instance(self, instance):
        for method in _get_public_methods(instance):
            name = method.im_func.func_name
            assert (name not in self.mocks) or (self.mocks[name].side_effect == self._noop_func)
            self.mocks[name] = Mock(side_effect=method)

    def unregister_instance(self, instance):
        for method in _get_public_methods(instance):
            name = method.im_func.func_name
            assert method == self.mocks[name].side_effect
            self.mocks[name].configure_mock(side_effect=self._noop_func)

    def _noop_func(self, *args, **kwargs):
        pass

    def _get_mock(self, name):
        try:
            return self.mocks[name]
        except KeyError:
            self.mocks[name] = val = Mock(wraps=self._noop_func)
            return val

    def _dispatch(self, method, params):
        mock_ = self._get_mock(method)
        if isinstance(params, dict):
            return mock_(**params)
        else:
            return mock_(*params)

mocker = Mocker()

class ArithA(object):
    def add(self, a, b):
        return a + b
    def div(self, x, y):
        return x / y
    def _private_method(self):
        pass
    _anyprivate = "dummy"

class ArithA2(object):
    def add(self, a, b):
        return a + b
    def div(self, x, y):
        return x / y

class ArithB(object):
    def mul(self, a, b):
        return a * b
    def sub(self, x, y):
        return x - y

def printme(verbose = False):
    if verbose:
        out = ['\n'.join((name, pformat(mock_.call_args_list))) for name, mock_ in mocker.mocks.items()]
        return '\n'.join(out)
    else:
        out = [name + ' ' + str(mock_.call_count) for name, mock_ in mocker.mocks.items()]
        return '\n'.join(out)

def givememoney():
    import pdb;pdb.set_trace()

inst = ArithA()
mocker.register_instance(inst)
mocker.register_instance(ArithB())
mocker.unregister_instance(inst)
mocker.register_instance(ArithA2())
mocker.register_function(printme)
mocker.register_function(givememoney)

server = SimpleJSONRPCServer(('localhost', 8080), logRequests=False)
server.register_instance(mocker)
print('Server listening on %s:%s' % server.socket.getsockname())
server.serve_forever()
