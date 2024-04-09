#!/usr/bin/env python

import sys
import pyautogui as pag
import common as cmn
import pyautogui as pag


pag.PAUSE = 0.017
pag.mouseDown(button=pag.RIGHT)
pag.mouseUp(button=pag.RIGHT)

pt = pag.position()
pag.moveTo(pag.Point(pt.x + 15, pt.y + 40))

pag.keyDown('shift')

pag.mouseDown(button=pag.LEFT)
pag.mouseUp(button=pag.LEFT)

pag.keyUp('shift')
