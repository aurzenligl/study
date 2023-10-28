#!/usr/bin/env python

import sys
import pyautogui as pag

c1 = pag.Point(50, 1000)
c2 = pag.Point(120, 1000)
c3 = pag.Point(50, 1050)
c4 = pag.Point(120, 1050)
h = pag.Point(275, 1030)

i = int(sys.argv[-1])
c = {1: c1, 2: c2, 3: c3, 4: c4}[i]

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(c); pag.leftClick()
pag.moveTo(h); pag.leftClick()
pag.moveTo(p)
