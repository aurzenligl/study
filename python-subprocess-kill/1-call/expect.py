'''
Check if process succeeds to satisfy condition within deadline,
times out or dies miserably. In latter cases utility cleans the
remnants from the process list.
'''

import subprocess
import shlex
import time
import os

def expect(cmd, cond, timeout):
    proc = subprocess.Popen(shlex.split(cmd), stderr=open(os.devnull))

    deadline = time.time() + timeout

    while time.time() < deadline:
        if cond(proc):
            return proc
        if proc.poll() is not None:
            raise Exception('process "%s" ended before satisfying condition' % cmd)
        time.sleep(0.1)

    proc.kill()
    proc.wait()
    raise Exception('process "%s" timed out before satisfying condition' % cmd)

def signal_file_exists(_):
    return os.path.exists('/tmp/tracer/signal0')

for t in [0.1, 2, 4]:
    try:
        proc = expect('./tracer.sh', signal_file_exists, timeout=t)
        proc.kill()
        print("success!")
        break
    except Exception as e:
        print("failure: %s" % e)
