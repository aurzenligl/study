'''
Safe and simple way to call process with timeout,
kill on timeout and return return-code, stdout and stderr.
'''

from threading import Timer
from subprocess import PIPE
import subprocess
import shlex

def run(cmd, timeout):
    proc = subprocess.Popen(shlex.split(cmd), stdout=PIPE, stderr=PIPE)
    timeout = Timer(timeout, lambda: proc.kill())

    try:
        timeout.start()
        out, err = proc.communicate()
    finally:
        timeout.cancel()
    ret = proc.poll()

    return ret, out, err

x = run('ping wp.pl -c1', timeout=3)

print('ret:', x[0])
print('out:', x[1])
print('err:', x[2])

