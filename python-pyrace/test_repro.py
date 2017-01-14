import os
import py
import pytest
import racefree

@pytest.fixture(autouse=True)
def patch_make_numbered_dir(monkeypatch):
    monkeypatch.setattr(py.path.local, 'make_numbered_dir', classmethod(racefree.make_numbered_dir))

@pytest.yield_fixture(scope='session')
def rootdir():
    yield py.path.local(os.path.join(os.path.dirname(__file__), 'rootdir'))

@pytest.mark.parametrize('_', 100 * [''])
def test_reproduce(rootdir, _):
    for i in range(100):
        tmpdir = py.path.local.make_numbered_dir(prefix='repro-', rootdir=rootdir)
        tmpdir.join('foo').write('bar')
        assert tmpdir.join('foo').read() == 'bar'
        tmpdir.remove()

# run this suite with xdist to reproduce:
# $ pytest -n10

# this line of code put after 'udir = rootdir.mkdir(prefix + str(maxnum+1))' increases chances of reproduction greatly:
# import time,random;time.sleep(random.random()/100)
