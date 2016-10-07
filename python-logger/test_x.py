import pytest
import logging

datalgr = logging.getLogger('data')
datalgr.addHandler(logging.NullHandler())

imlgr = logging.getLogger('im')
imlgr.addHandler(logging.NullHandler())

problemlgr = logging.getLogger('problem')
problemlgr.addHandler(logging.NullHandler())

def test_foo(the_error_fixture):
    datalgr.warning('data -1')
    datalgr.warning('connection available')
    datalgr.warning('data 0')

    imlgr.warning('object ABC')
    imlgr.warning('object DEF')

    datalgr.warning('data 1')

def test_bar(the_error_fixture, tmpdir, logdir):
    datalgr.warning('data -1')
    datalgr.warning('connection available')
    datalgr.warning('data 0')

    imlgr.warning('object ABC')

    if the_error_fixture:
        problemlgr.warning("we're about to crash")
        assert 1 != 1

    imlgr.warning('object DEF')

    datalgr.warning('data 1')
