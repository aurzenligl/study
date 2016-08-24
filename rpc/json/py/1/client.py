#!/usr/bin/env python

import sys
import jsonrpclib

def main():
    server = jsonrpclib.Server('http://localhost:8080')

    for _ in range(3):
        print server.add(123, 456)

    print jsonrpclib.history.request
    print jsonrpclib.history.response

    batch = jsonrpclib.MultiCall(server)
    batch.add(5, 6)
    batch.ping({'key':'value'})
    results = batch()
    print([x for x in results])

if __name__ == '__main__':
    main()
