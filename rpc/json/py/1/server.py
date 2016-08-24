#!/usr/bin/env python

from jsonrpclib.SimpleJSONRPCServer import SimpleJSONRPCServer

server = SimpleJSONRPCServer(('localhost', 8080))
server.register_function(pow)
server.register_function(lambda x,y: x+y, 'add')
server.register_function(lambda x: x, 'ping')

print('Server listening on %s:%s' % server.socket.getsockname())
server.serve_forever()
