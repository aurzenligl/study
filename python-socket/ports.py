import os
import atexit
import socket
import fasteners
import subprocess
import time

# make PortAllocator or IdAllocator out of it 

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
    while True:
        port = reserve_id(lockfile, first)
        if not is_port_free(port):
            continue
        return port

def free_port(lockfile, port):
    free_id(lockfile, port)

def reserve_id(lockfile, first, lock_timeout=172800):  # two days
    with fasteners.InterProcessLock(lockfile) as f:
        db = read_db(f.lockfile.name)
        db = prune_db(db, lock_timeout)
        id = reserve_db(db, first)
        save_db(db, f.lockfile)

    # id will be removed at process exit
    mypid = os.getpid()
    def try_remove_id():
        # in a fork() situation, only the last process should
        # remove the id, otherwise the other processes run the
        # risk of seeing their temporary file disappear.  For now
        # we remove the id in the parent only (i.e. we assume
        # that the children finish before the parent).
        if os.getpid() != mypid:
            return
        try:
            if str(id) in open(lockfile).read():
                free_id(lockfile, id)
        except:
            pass
    atexit.register(try_remove_id)

    return id

def free_id(lockfile, port):
    with fasteners.InterProcessLock(lockfile) as f:
        db = read_db(lockfile)
        db = remove_db(db, port)
        save_db(db, f.lockfile)

def read_db(name):
    x = subprocess.check_output(['cat', name])
    db = [y.split() for y in x.splitlines()]
    return db

def prune_db(db, timeout):
    def is_not_timed_out(record):
        t1 = float(record[1])
        t2 = time.time()
        return abs(t2 - t1) < timeout
    return filter(is_not_timed_out, db)

def reserve_db(db, first):
    used = set(int(p[0]) for p in db)
    port = first
    while port in used:
        port += 1
    db.append([port, time.time()])
    return port

def save_db(db, f):
    def to_line(record):
        return ' '.join((str(x) for x in record))
    def to_content(db):
        return ''.join(to_line(rec) + '\n' for rec in db)
    f.truncate(0)
    f.write(to_content(db))

def remove_db(db, port):
    def is_not_port(record):
        return int(record[0]) != port
    return filter(is_not_port, db)
