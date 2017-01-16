import os
import atexit
import py

def make_numbered_dir(cls, prefix='session-', rootdir=None, keep=3,
                      lock_timeout = 172800):   # two days
    """ return unique directory with a number greater than the current
        maximum one.  The number is assumed to start directly after prefix.
        if keep is true directories with a number less than (maxnum-keep)
        will be removed.
    """
    if rootdir is None:
        rootdir = cls.get_temproot()

    def parse_num(path):
        """ parse the number out of a path (if it matches the prefix) """
        bn = path.basename
        if bn.startswith(prefix):
            try:
                return int(bn[len(prefix):])
            except ValueError:
                pass

    # compute the maximum number currently in use with the
    # prefix
    lastmax = None
    while True:
        maxnum = -1
        for path in rootdir.listdir():
            num = parse_num(path)
            if num is not None:
                maxnum = max(maxnum, num)

        # make the new directory
        try:
            udir = rootdir.mkdir(prefix + str(maxnum+1))
        except py.error.EEXIST:
            # race condition: another thread/process created the dir
            # in the meantime.  Try counting again
            if lastmax == maxnum:
                raise
            lastmax = maxnum
            continue
        break

    # put a .lock file in the new directory that will be removed at
    # process exit
    if lock_timeout:
        lockfile = udir.join('.lock')
        mypid = os.getpid()
        if hasattr(lockfile, 'mksymlinkto'):
            lockfile.mksymlinkto(str(mypid))
        else:
            lockfile.write(str(mypid))
        def try_remove_lockfile():
            # in a fork() situation, only the last process should
            # remove the .lock, otherwise the other processes run the
            # risk of seeing their temporary dir disappear.  For now
            # we remove the .lock in the parent only (i.e. we assume
            # that the children finish before the parent).
            if os.getpid() != mypid:
                return
            try:
                lockfile.remove()
            except py.error.Error:
                pass
        atexit.register(try_remove_lockfile)

    # prune old directories
    if keep:
        def get_mtime(pth):
            try:
                return pth.lstat().mtime
            except py.error.Error:
                pass

        t2 = get_mtime(udir)

        for path in rootdir.listdir():
            num = parse_num(path)
            if num is not None and num <= (maxnum - keep):

                t1 = get_mtime(path)
                has_lock = path.join('.lock').exists()

                if not t1 or not t2:
                    continue
                if has_lock and lock_timeout and abs(t2-t1) < lock_timeout:
                    continue
                if abs(t2-t1) < 1:
                    continue

                try:
                    path.remove(rec=1)
                except KeyboardInterrupt:
                    raise
                except: # this might be py.error.Error, WindowsError ...
                    pass

    # make link...
    try:
        username = os.environ['USER']           #linux, et al
    except KeyError:
        try:
            username = os.environ['USERNAME']   #windows
        except KeyError:
            username = 'current'

    src  = str(udir)
    dest = src[:src.rfind('-')] + '-' + username
    try:
        os.unlink(dest)
    except OSError:
        pass
    try:
        os.symlink(src, dest)
    except (OSError, AttributeError, NotImplementedError):
        pass

    return udir
