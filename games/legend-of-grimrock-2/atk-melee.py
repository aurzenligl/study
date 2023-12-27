#!/usr/bin/env python

import pyautogui as pag
import common as cmn

c0 = cmn.Char(0)
c1 = cmn.Char(1)
c2 = cmn.Char(2)
p = pag.position()
cmn.rclick(c0.slot(0))
cmn.rclick(c0.slot(1))
cmn.rclick(c1.slot(0))
cmn.rclick(c2.slot(0))
cmn.rclick(c2.slot(1))
pag.moveTo(p)
