#!/usr/bin/env python

import pyautogui as pag

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(1860, 1030)
pag.rightClick()
pag.moveTo(1815, 970)
pag.rightClick()
pag.moveTo(1860, 1030)
pag.rightClick()
pag.moveTo(p)
