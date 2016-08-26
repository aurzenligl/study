#!/usr/bin/env python

from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer
from bunch import Bunch
from mock import Mock
from pprint import pformat

def resolve_attribute(obj, attr):
    return getattr(obj, i)

class Mocker(object):
    def __init__(self):
        self.mocks = Bunch()
        self._instances = []

    def register_function(self, function):
        mock_ = self._get_mock(function.__name__)
        mock_.configure_mock(_mock_wraps=function)

    def unregister_function(self, function):
        mock_ = self._get_mock(function.__name__)
        mock_.configure_mock(_mock_wraps=self._noop_func)

    def register_instance(self, instance):
        self._instances.append(instance)
        for name, mock_ in self.mocks.items():
            obj = getattr(instance, name, None)
            if obj is not None:
                mock_.configure_mock(_mock_wraps=obj)

    def unregister_instance(self, instance):
        self._instances.remove(instance)
        for name, mock_ in self.mocks.items():
            obj = getattr(instance, name, None)
            if obj is not None:
                mock_.configure_mock(_mock_wraps=self._noop_func)

    def _noop_func(self, *args, **kwargs):
        pass

    def _resolve_instance_attribute(self, attr):
        for inst in reversed(self._instances):
            obj = getattr(inst, attr, None)
            if obj is not None:
                return obj

    def _get_mock(self, name):
        try:
            return self.mocks[name]
        except KeyError:
            obj = self._resolve_instance_attribute(name)
            self.mocks[name] = val = Mock(wraps=(obj or self._noop_func))
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

mocker.register_instance(ArithA())
mocker.register_instance(ArithB())
mocker.register_function(printme)
mocker.register_function(givememoney)

server = SimpleJSONRPCServer(('localhost', 8080), logRequests=False)
server.register_instance(mocker)
print('Server listening on %s:%s' % server.socket.getsockname())
server.serve_forever()
