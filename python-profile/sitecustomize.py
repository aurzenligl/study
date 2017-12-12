import os
import atexit
import cProfile

prof = cProfile.Profile()
prof.enable()
def fin():
    prof.disable()
    prof.dump_stats('%s.cprof' % os.getpid())
atexit.register(fin)

# * * *

import threading, sys, traceback, signal

def dumpstacks(signal, frame):
    id2name = dict([(th.ident, th.name) for th in threading.enumerate()])
    code = []
    for threadId, stack in sys._current_frames().items():
        code.append("\n# Thread: %s(%d)" % (id2name.get(threadId,""), threadId))
        for filename, lineno, name, line in traceback.extract_stack(stack):
            code.append('File: "%s", line %d, in %s' % (filename, lineno, name))
            if line:
                code.append("  %s" % (line.strip()))
    print "\n".join(code)

signal.signal(signal.SIGUSR1, dumpstacks)
