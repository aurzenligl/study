import os
import atexit
import cProfile

prof = cProfile.Profile()
prof.enable()
def fin():
    prof.disable()
    prof.dump_stats('%s.cprof' % os.getpid())
atexit.register(fin)
