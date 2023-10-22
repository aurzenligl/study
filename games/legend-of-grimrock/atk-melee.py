#!/usr/bin/env python

import pyautogui as pag

pag.PAUSE = 0.02
p = pag.position()
pag.moveTo(1750, 850)
pag.rightClick()
pag.moveTo(1530, 850)
pag.rightClick()
pag.moveTo(p)
