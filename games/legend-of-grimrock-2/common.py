import pyautogui as pag

class Point(pag.Point):
    def __add__(self, rhs):
        return Point(self.x + rhs.x, self.y + rhs.y)
    def __mul__(self, rhs):
        return Point(self.x * rhs, self.y * rhs)
    def __rmul__(self, rhs):
        return Point(self.x * rhs, self.y * rhs)

class Char:
    def __init__(self, char):
        y, x = divmod(char, 2)
        self.char = char
        self.origin = origin + cdx * x + cdy * y

    def slot(self, slot):
        return self.origin + sdx * slot + sc

    def rune(self, rune):
        j, i = divmod(rune, 3)
        return self.origin + rdx * i + rdy * j + rc

    def cast(self):
        return self.origin + cstc

    def cancel(self):
        return self.origin + cncc

def rclick(pt):
    pag.moveTo(pt)
    pag.mouseDown(pt, button=pag.RIGHT)
    pag.mouseUp(pt, button=pag.RIGHT)

def rdrag(pts):
    pag.moveTo(pts[0])
    pag.mouseDown(pts[0], button=pag.RIGHT)
    for pt in pts:
        pag.moveTo(pt)
        pag.moveTo(pt)
    pag.mouseUp(pts[-1], button=pag.RIGHT)

pag.PAUSE = 0.017
origin = Point(1493, 742)
cdx = Point(206, 0)  # char dx
cdy = Point(0, 200)  # char dy
sdx = Point(83, 0)  # slot dx
sc = Point(37, 51)  # slot center
rdx = Point(44, 0)  # rune dx
rdy = Point(0, 38)  # rune dy
rc = Point(17, 12)  # rune center
cstc = Point(145, 63)  # cast center
cncc = Point(145, 4)  # cancel center
