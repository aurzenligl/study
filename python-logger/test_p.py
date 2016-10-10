import logging
from process import process

proclgr = logging.getLogger('proc')
proclgr.addHandler(logging.NullHandler())

imlgr = logging.getLogger('im')
imlgr.addHandler(logging.NullHandler())

def test_echo():
    with process('echo abc def', stdoutlgr=proclgr) as proc:
        import time; time.sleep(0.1)

def test_cat():
    with process('cat okaowq', stderrlgr=proclgr) as proc:
        import time; time.sleep(0.1)

def test_qez(daemon):
    imlgr.info('object A')
    import time;time.sleep(0.1)
    imlgr.info('object B')
    import time;time.sleep(0.1)
    imlgr.info('object C')
