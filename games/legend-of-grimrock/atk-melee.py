#!/usr/bin/env python

import pyautogui as pag

rx = [1530, 1750]
ry = [ 850, 1020]
r1 = pag.Point(rx[0], ry[0])
r2 = pag.Point(rx[1], ry[0])
r3 = pag.Point(rx[0], ry[1])
r4 = pag.Point(rx[1], ry[1])

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(r1); pag.rightClick()
pag.moveTo(r2); pag.rightClick()
# pag.moveTo(r3); pag.rightClick()
# pag.moveTo(r4); pag.rightClick()
# pag.moveTo(r3); pag.rightClick()
pag.moveTo(p)
