import shlex
from subprocess import Popen, PIPE
from contextlib import contextmanager
from threading import Thread

@contextmanager
def process(cmd, stdoutlgr=None, stderrlgr=None):
    proc = Process(cmd, stdoutlgr, stderrlgr)
    try:
        yield proc
    finally:
        proc.stop()

class Process(object):
    def __init__(self, cmd, stdoutlgr, stderrlgr):
        # executing via shell causes difficulties with killing child process
        # http://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true
        # http://stackoverflow.com/questions/1624254/how-do-i-close-the-stdout-pipe-when-killing-a-process-started-with-python-subpro
        self.popen = Popen(shlex.split("stdbuf -oL -eL " + cmd), stdout=PIPE, stderr=PIPE)
        self.stdoutthr = stdoutlgr and start_consumer_thread(self.popen.stdout, stdoutlgr) or None
        self.stderrthr = stderrlgr and start_consumer_thread(self.popen.stderr, stderrlgr) or None
    def stop(self):
        self.popen.terminate()
        if self.stdoutthr:
            self.stdoutthr.join()
        if self.stderrthr:
            self.stderrthr.join()

def start_consumer_thread(pipe, lgr):
    thr = Thread(target=consume_lines, args=[pipe, lgr])
    thr.start()
    return thr

def consume_lines(pipe, lgr):
    with pipe:
        for line in iter(pipe.readline, b''):  #workaround read-ahead bug
            lgr.info(line.rstrip('\n'))
