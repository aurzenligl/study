import pytest
import logging
import thelib

datalgr = logging.getLogger('data')
datalgr.addHandler(logging.NullHandler())

imlgr = logging.getLogger('im')
imlgr.addHandler(logging.NullHandler())

problemlgr = logging.getLogger('problem')
problemlgr.addHandler(logging.NullHandler())

def test_foo(the_error_fixture):
    datalgr.info('data -1')
    datalgr.info('connection available')
    datalgr.info('data 0')

    imlgr.info('object ABC')
    imlgr.info('object DEF')

    datalgr.info('data 1')

    thelib.fbar.ffoo('sdda')
    thelib.fbar.fbar(1, 2)

def test_bar(the_error_fixture, tmpdir, logdir):
    datalgr.info('data -1')
    datalgr.info('connection available')
    datalgr.info('data 0')

    imlgr.info('object ABC')

    if the_error_fixture:
        problemlgr.info("we're about to crash")
        assert 1 != 1

    imlgr.info('object DEF')

    datalgr.info('data 1')

    thelib.fbar.ffoo(a=5)
    thelib.fbar.fbar(a='1', b='2')

def test_thelib():
    thelib.cfoo.Foo(42)
    thelib.cfoo.Foo(a=42)

    foo = thelib.cfoo.Foo()
    foo.foo('ghs')
    foo.bar(1,2,3,4)
    foo.sfoo(a=[5,4,3])
    foo.cfoo(a='abc')
