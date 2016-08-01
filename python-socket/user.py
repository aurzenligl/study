#!/usr/bin/env python

import sys
import socket
import time

def tst():
    import os
    import time
    import datetime
    pid = os.getpid()
    ts = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')
    return "[%s] %s:" % (ts, pid)

def do(port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("", port))
        s.listen(1)
        print('[%s] success: used port %s' % (tst(), port))
        return s
    except socket.error as e:
        print('[%s] error: on port %s, %s' % (tst(), port, e))

ports = [int(x) for x in sys.argv[1:]]
sockets = [do(port) for port in ports]
time.sleep(0.5)
