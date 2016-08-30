#!/usr/bin/env python

import sys
import jsonrpclib

def main():
    server = jsonrpclib.Server('http://localhost:8080')

    for _ in range(1):
        print server.add(a=30, b=6)
        print server.sub(30, 6)
        print server.mul(a=30, b=6)
        print server.div(30, 6)

    if 0:
        print jsonrpclib.history.request
        print jsonrpclib.history.response

    if 1:
        server.trythisone()

    if 0:
        batch = jsonrpclib.MultiCall(server)
        batch.add(5, 6)
        batch.ping({'key':'value'})
        results = batch()
        print([x for x in results])

    if 0:
        print server.printme()
        print server.printme(verbose=True)

    if 0:
        server.givememoney()

if __name__ == '__main__':
    main()
