#!/usr/bin/env python

import sys
import socket
import time

def do(port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("", port))
        s.listen(1)
        print('success: used port %s' % port)
        return s
    except socket.error as e:
        print('error: on port %s, %s' % (port, e))

ports = [int(x) for x in sys.argv[1:]]
sockets = [do(port) for port in ports]
time.sleep(0.5)