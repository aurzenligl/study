import sys
import subprocess

def foo(x):
    return x.index(42)

xx = [x for x in range(100000)]
foo(xx)

if 'subpro' in sys.argv:
    sys.exit(0)

subprocess.Popen('python spike.py subpro', shell=True)
