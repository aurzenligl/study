#!/usr/bin/env python

import pyautogui as pag
import common as cmn

pag.PAUSE = 0.02
p = pag.position()
cmn.rclick(cmn.slot(0, 0))
cmn.rclick(cmn.slot(1, 0))
pag.moveTo(p)
