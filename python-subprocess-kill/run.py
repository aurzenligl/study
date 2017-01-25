#!/usr/bin/env python

import subprocess
from contextlib import contextmanager
import shlex

@contextmanager
def proc(cmd):
    popen = subprocess.Popen(shlex.split(cmd))
    try:
        yield popen
    finally:
        popen.kill()
        popen.wait()

def do():
    with proc('bwrap --bind / / ./foo'):
        import time;time.sleep(10)

if __name__ == '__main__':
    do()
