#!/usr/bin/env python

import sys
import pyautogui as pag
import common as cmn

opts = sys.argv[1:]
if not len(opts):
    sys.exit(1)

name = opts.pop(0)
if name == 'fireburst':
    pag.PAUSE = 0.02
    p = pag.position()
    cmn.rclick(cmn.slot(2, 0))
    cmn.rclick(cmn.rune(2, 0))
    cmn.rclick(cmn.cast(2))
    cmn.rclick(cmn.cancel(2))
    pag.moveTo(p)
