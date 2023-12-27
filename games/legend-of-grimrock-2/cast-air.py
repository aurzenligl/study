#!/usr/bin/env python

import sys
import pyautogui as pag
import common as cmn

opts = sys.argv[1:]
if not len(opts):
    sys.exit(1)

name = opts.pop(0)
if name == 'shock':
    c = cmn.Char(3)
    p = pag.position()
    cmn.rclick(c.slot(0))
    cmn.rdrag([c.rune(2)])
    cmn.rclick(c.cast())
    cmn.rclick(c.cancel())
    pag.moveTo(p)
elif name == 'lightning-bolt':
    c = cmn.Char(3)
    p = pag.position()
    cmn.rclick(c.slot(0))
    cmn.rdrag([c.rune(3), c.rune(4), c.rune(1), c.rune(2)])
    cmn.rclick(c.cast())
    cmn.rclick(c.cancel())
    pag.moveTo(p)
