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

    mypid = os.getpid()

    def schedule_lockfile_removal(lockfile):
        """ ensure lockfile is removed at process exit """

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

    def get_mtime(path):
        """ read file modification time """
        try:
            return path.lstat().mtime
        except py.error.Error:
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
            if lock_timeout:
                lockfile = udir.join('.lock')
                lockfile.write(str(mypid), 'wx')
                schedule_lockfile_removal(lockfile)
        except (py.error.EEXIST, py.error.ENOENT):
            # race condition (1): another thread/process created the dir
            # in the meantime.  Try counting again
            # race condition (2): another thread/process acquired lock
            # treating empty directory as candidate for removal.  Try again
            if lastmax == maxnum:
                raise
            lastmax = maxnum
            continue
        break

    # prune old directories
    t2 = get_mtime(udir)
    if keep and lock_timeout and t2:
        for path in rootdir.listdir():
            num = parse_num(path)
            if num is not None and num <= (maxnum - keep):
                try:
                    # try acquiring lock to remove directory as exclusive user
                    path.join('.lock').write(str(mypid), 'wx')
                except (py.error.EEXIST, py.error.ENOENT):
                    t1 = get_mtime(path)
                    if not t1:
                        continue
                    if abs(t2-t1) < lock_timeout:
                        continue

                # rename dir scheduled for removal to avoid another thread/process
                # treating it as a new directory or to-remove directory
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
