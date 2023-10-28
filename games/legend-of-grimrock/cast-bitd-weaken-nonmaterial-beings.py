#!/usr/bin/env python

import pyautogui as pag

def shl(pt):
  return pag.Point(pt.x - 220, pt.y)

c0 = pag.Point(1760, 1030)
c1 = pag.Point(1860, 1030)
rx = [1725, 1770, 1815]
ry = [ 970,  995, 1040]
r1 = pag.Point(rx[0], ry[0])
r2 = pag.Point(rx[1], ry[0])
r3 = pag.Point(rx[2], ry[0])
r4 = pag.Point(rx[0], ry[1])
r5 = pag.Point(rx[1], ry[1])
r6 = pag.Point(rx[2], ry[1])
r7 = pag.Point(rx[0], ry[2])
r8 = pag.Point(rx[1], ry[2])
r9 = pag.Point(rx[2], ry[2])

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(c0); pag.rightClick()
pag.moveTo(r4); pag.rightClick()
pag.moveTo(r8); pag.rightClick()
pag.moveTo(c1); pag.rightClick()
pag.moveTo(shl(c0)); pag.rightClick()
pag.moveTo(shl(r4)); pag.rightClick()
pag.moveTo(shl(r8)); pag.rightClick()
pag.moveTo(shl(c1)); pag.rightClick()
pag.moveTo(p)
