import logging
import os
import sys
import time

logger = logging.getLogger('setup')
logger.addHandler(logging.NullHandler())

class Luger:
    def __init__(self, path):
        self.path = path
        self.pid = os.getpid()
        try:
            os.mkdir(path)
        except:
            pass
        self.put('-- starting %s --' % self.pid)
    def put(self, line):
        with open('%s/%s' % (self.path, self.pid), 'a') as f:
            f.write('%.4f: %s\n' % (time.time(), line))

luger = Luger('/tmp/luger/')

def put(line):
    luger.put(line)
