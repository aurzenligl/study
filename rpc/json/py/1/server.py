#!/usr/bin/env python

from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer

server = SimpleJSONRPCServer(('localhost', 8080), logRequests=False)
server.register_function(pow)
server.register_function(lambda x,y: x+y, 'add')
server.register_function(lambda x: x, 'ping')

# use mocks
# use bunch

# mocker.register_function(func)
# mocker.unregister_function(func)
# mocker.register_instance(inst)
# mocker.unregister_instance(inst)
# with mocker.with_function(func): pass
# with mocker.with_instance(inst): pass

# mocker.mocks is a bunch of mocks, lazily added during calls
# register func/instance add/remove wrapper functions for calls

# mocker.mocks.vtc__do_it
# mocker.mocks
# mocker.with_func(vtc__foobar)

# mocker.mocks.vtc
# '_mock_wraps': <function add at 0x7ffb5f326668>,

class Mocker(object):
    def register_function(self, function, name = None):
        pass
    def unregister_function(self, function, name = None):
        pass
    def _precall(self, method, params):
        # call mock here
        print('pre method: %s was called with params: %s' % (method, params))
    def _unknowncall(self, method, params):
        # return None here
        print('unknown method: %s was called with params: %s' % (method, params))
    def _dispatch(self, method, params):

        func = None
        try:
            func = self.funcs[method]
        except KeyError:
            for instance in self.instances:
                try:
                    func = resolve_dotted_attribute(
                        self.instance,
                        method,
                        self.allow_dotted_names
                        )
                    break
                except AttributeError:
                    pass

        if func is not None:
            return func(*params)
        else:
            raise Exception('method "%s" is not supported' % method)

server.register_instance(Mocker())

print('Server listening on %s:%s' % server.socket.getsockname())
server.serve_forever()
