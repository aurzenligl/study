#!/usr/bin/env python

import pytest
import logging

datalgr = logging.getLogger('data')
datalgr.addHandler(logging.NullHandler())

imlgr = logging.getLogger('im')
imlgr.addHandler(logging.NullHandler())

def test_foo(the_error_fixture):
    datalgr.warning('data -1')
    datalgr.warning('connection available')
    datalgr.warning('data 0')

    imlgr.warning('object ABC')
    imlgr.warning('object DEF')

    datalgr.warning('data 1')

def test_bar(the_error_fixture):
    datalgr.warning('data -1')
    datalgr.warning('connection available')
    datalgr.warning('data 0')

    imlgr.warning('object ABC')

    if the_error_fixture:
        assert 1 != 1

    imlgr.warning('object DEF')

    datalgr.warning('data 1')
