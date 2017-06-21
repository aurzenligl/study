import os
import atexit
import py
import uuid

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

    removingprefix = prefix + 'removing-'

    def is_removing(path):
        """ check if path denotes directory scheduled for removal """
        bn = path.basename
        return bn.startswith(removingprefix)

    '''TODO bring back atexit .lock removal'''
    mypid = os.getpid()

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
            if lock_timeout:
                udir.join('.lock').write(str(mypid), 'wx')
        except (py.error.EEXIST, py.error.ENOENT):
            # race condition: another thread/process created the dir
            # in the meantime.  Try counting again
            if lastmax == maxnum:
                raise
            lastmax = maxnum
            continue
        break

    # prune old directories
    if lock_timeout and keep:
        try:
            t2 = udir.lstat().mtime
        except py.error.Error:
            pass
        if t2:
            for path in rootdir.listdir():
                num = parse_num(path)
                if num is not None and num <= (maxnum - keep):
                    try:
                        t1 = path.lstat().mtime
                    except py.error.Error:
                        continue
                    try:
                        path.join('.lock').write(str(mypid), 'wx')
                    except (py.error.EEXIST, py.error.ENOENT):
                        if abs(t2-t1) < lock_timeout:
                            continue

                    newname = removingprefix + str(uuid.uuid4())
                    renamed = rootdir.join(newname)
                    try:
                        path.rename(renamed)
                        renamed.remove(rec=1)
                    except KeyboardInterrupt:
                        raise
                    except: # this might be py.error.Error, WindowsError ...
                        pass
                if is_removing(path):
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
