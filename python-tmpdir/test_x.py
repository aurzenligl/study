import py
import pytest

class ParametricTempdirFactory:
    """Factory for temporary directories under the common base temp directory.

    A modified TempdirFactory from pytest framework.
    It allows user to provide common base temp directory explicitly
    and creates numbered subdirectories per test session.
    """

    def __init__(self, temproot):
        self.temproot = temproot

    def mktemp(self, basename):
        return self.getbasetemp().mkdir(basename)

    def getbasetemp(self):
        try:
            return self._basetemp
        except AttributeError:
            temproot = py.path.local(self.temproot)
            rootdir = temproot.join('pytest-of-%s' % get_user())
            rootdir.ensure(dir=1)
            basetemp = py.path.local.make_numbered_dir(prefix='pytest-', rootdir=rootdir)
            self._basetemp = t = basetemp.realpath()
            return t

def get_user():
    import getpass
    return getpass.getuser()

@pytest.fixture(scope='session')
def devshm_tmpdir_factory():
    return ParametricTempdirFactory('/dev/shm')

##################################

from time import sleep
N = 1

@pytest.yield_fixture(scope = 'session')
def my_shmdir(devshm_tmpdir_factory):
    yield devshm_tmpdir_factory.mktemp('my_shmdir')

@pytest.mark.parametrize('_', 20 * [''])
def test_x(tmpdir, my_shmdir, _):
    sleep(N)

#echo -e '1\n2\n3\n4\n5\n6' | parallel "py.test -m {}"
#tree /dev/shm

#echo -e '1\n2\n3\n4\n5\n6' | parallel "py.test -n 3 -m {}"
#tree /dev/shm
