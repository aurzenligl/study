import os
import atexit
import socket
import fasteners
import subprocess
import time

def get_free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("",0))
    s.listen(1)
    port = s.getsockname()[1]
    s.close()
    return port

def is_port_free(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("", port))
        s.listen(1)
        s.close()
        return True
    except socket.error as e:
        return False

def reserve_port(lockfile, first):
    pers = Allocator(lockfile)
    while True:
        port = pers.allocate(first)
        if not is_port_free(port):
            continue
        return port

def free_port(lockfile, port):
    pers = Allocator(lockfile)
    pers.free(port)

class Allocator(object):
    def __init__(self, path):
        self.path = path

    def allocate(self, first, lock_timeout=172800):  # two days
        with fasteners.InterProcessLock(self.path) as f:
            db = Db(self.path)
            db.prune(lock_timeout)
            id = db.reserve(first)
            db.save(f.lockfile)
        atexit.register(self._try_remove_id, id=id, pid=os.getpid(), path=self.path)
        return id

    def free(self, port):
        with fasteners.InterProcessLock(self.path) as f:
            db = Db(self.path)
            db.remove(port)
            db.save(f.lockfile)

    def _try_remove_id(self, id, pid, path):
        # in a fork() situation, only the last process should
        # remove the id, otherwise the other processes run the
        # risk of seeing their temporary file disappear.  For now
        # we remove the id in the parent only (i.e. we assume
        # that the children finish before the parent).
        if os.getpid() != pid:
            return
        try:
            if str(id) in open(self.path).read():
                self.free(id)
        except IOError:
            pass

class Db(object):
    def __init__(self, path):
        data = subprocess.check_output(['cat', path])
        self.db = [y.split() for y in data.splitlines()]

    def save(self, file_):
        def to_line(record):
            return ' '.join((str(x) for x in record))
        def to_content(db):
            return ''.join(to_line(rec) + '\n' for rec in db)
        file_.truncate(0)
        file_.write(to_content(self.db))

    def prune(self, timeout):
        def is_not_timed_out(record):
            t1 = float(record[1])
            t2 = time.time()
            return abs(t2 - t1) < timeout
        self.db = filter(is_not_timed_out, self.db)

    def reserve(self, first):
        used = set(int(p[0]) for p in self.db)
        port = first
        while port in used:
            port += 1
        self.db.append([port, time.time()])
        return port

    def remove(self, port):
        def is_not_port(record):
            return int(record[0]) != port
        self.db = filter(is_not_port, self.db)
