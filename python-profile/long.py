import os
import time

print(os.getpid())

def bar():
    time.sleep(2)

def foo():
    time.sleep(1)
    bar()
    time.sleep(1)

[foo() for _ in range(100)]

