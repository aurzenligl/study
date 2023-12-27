#!/usr/bin/env python

import sys
import pyautogui as pag
import common as cmn

opts = sys.argv[1:]
if not len(opts):
    sys.exit(1)

name = opts.pop(0)
if name == 'light':
    c = cmn.Char(3)
    p = pag.position()
    cmn.rclick(c.slot(0))
    cmn.rdrag([c.rune(1), c.rune(4)])
    cmn.rclick(c.cast())
    cmn.rclick(c.cancel())
    pag.moveTo(p)
if name == 'force-field':
    c = cmn.Char(3)
    p = pag.position()
    cmn.rclick(c.slot(0))
    cmn.rdrag([c.rune(0), c.rune(1), c.rune(2), c.rune(5), c.rune(8), c.rune(7), c.rune(6), c.rune(3), c.rune(0)])
    cmn.rclick(c.cast())
    cmn.rclick(c.cancel())
    pag.moveTo(p)
