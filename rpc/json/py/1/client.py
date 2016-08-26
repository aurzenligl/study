#!/usr/bin/env python

import sys
import jsonrpclib

def main():
    server = jsonrpclib.Server('http://localhost:8080')

#     server.givememoney()

    print server.add(a=30, b=6)
    print server.sub(30, 6)
    print server.mul(a=30, b=6)
    print server.div(30, 6)

    print jsonrpclib.history.request
    print jsonrpclib.history.response

    batch = jsonrpclib.MultiCall(server)
    batch.add(5, 6)
    batch.ping({'key':'value'})
    results = batch()
    print([x for x in results])

    print server.printme()
    print server.printme(verbose=True)

if __name__ == '__main__':
    main()
