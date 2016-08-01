import py
import os
import atexit
import socket
import portalocker

# make PortAllocator or IdAllocator out of it 

def tst():
    import os
    import time
    import datetime
    pid = os.getpid()
    ts = datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S.%f')
    return "[%s] %s:" % (ts, pid)

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

def reserve_port():
    while True:
        portfile = make_numbered_file('/tmp/pytest-sockets', 20000)
        port = int(portfile.basename)
        if not all(is_port_free(port) for port in (port, port, port)):
            print('[%s] reserve: NOK %s' % (tst(), port))
            continue
        print('[%s] reserve: OK %s' % (tst(), port))
        return port

def free_port(port):
    ufile = py.path.local('/tmp/pytest-sockets').join(str(port))
    try:
        print('[%s] ports: -%s' % (tst(), ufile))
        ufile.remove()
    except py.error.Error:
        pass

def make_numbered_file(rootdir, first=0,
                       lock_timeout = 172800):   # two days
    lockfile = py.path.local(rootdir).join('lock')
    with portalocker.Lock(str(lockfile), fail_when_locked=False):
        return _make_numbered_file(rootdir, first, lock_timeout)

def _make_numbered_file(rootdir, first=0,
                        lock_timeout = 172800):   # two days
    """ return unique file with a first free number greater or equal to given one as name
    """
    rootdir = py.path.local(rootdir).ensure(dir=1)

    def parse_num(path):
        """ parse the number out of a path (if it matches the prefix) """
        bn = path.basename
        try:
            return int(bn)
        except ValueError:
            pass

    # compute the first free file number
    while True:
        nums = []
        for path in rootdir.listdir():
            num = parse_num(path)
            if num is not None:
                nums.append(num)

        next_num = first
        while True:
            if next_num in nums:
                next_num += 1
                continue
            break

        # make the new file
        try:
            ufile = rootdir.join(str(next_num))
            with portalocker.Lock(str(ufile)):
                pass
        except portalocker.AlreadyLocked:
            # race condition: another thread/process created the file
            # in the meantime.  Try counting again
            print('[%s] ports: =%s' % (tst(), ufile))
            continue
        # a bug! something is wrong with this portalocker:
        # [[2016-08-01 04:59:57.126145] 30508:] ports: +/tmp/pytest-sockets/20012
        # [[2016-08-01 04:59:57.126164] 30511:] ports: +/tmp/pytest-sockets/20012
        print('[%s] ports: +%s' % (tst(), ufile))
        break

    # file will be removed at process exit
    mypid = os.getpid()
    def try_remove_file():
        # in a fork() situation, only the last process should
        # remove the file, otherwise the other processes run the
        # risk of seeing their temporary file disappear.  For now
        # we remove the file in the parent only (i.e. we assume
        # that the children finish before the parent).
        if os.getpid() != mypid:
            return
        try:
            ufile.remove()
        except py.error.Error:
            pass
    atexit.register(try_remove_file)

    # prune old files
    for path in rootdir.listdir():
        num = parse_num(path)
        if num is not None:
            try:
                t1 = path.lstat().mtime
                t2 = ufile.lstat().mtime
                if abs(t2-t1) < lock_timeout:
                    continue   # skip files not timed out
            except py.error.Error:
                pass
            try:
                path.remove()
            except KeyboardInterrupt:
                raise
            except: # this might be py.error.Error, WindowsError ...
                pass

    return ufile
