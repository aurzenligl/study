#!/usr/bin/env python

import pyautogui as pag

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(1530, 1020)
pag.rightClick()
pag.moveTo(p)

# pag.moveTo(1615, 1020)
# pag.rightClick()
