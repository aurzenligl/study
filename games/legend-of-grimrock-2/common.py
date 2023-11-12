import pyautogui as pag

class Point(pag.Point):
    def __add__(self, rhs):
        return Point(self.x + rhs.x, self.y + rhs.y)
    def __mul__(self, rhs):
        return Point(self.x * rhs, self.y * rhs)
    def __rmul__(self, rhs):
        return Point(self.x * rhs, self.y * rhs)

def slot(char, slot):
    y, x = divmod(char, 2)
    return origin + cdx * x + cdy * y + sdx * slot + sc

def rune(char, rune):
    y, x = divmod(char, 2)
    j, i = divmod(rune, 3)
    return origin + cdx * x + cdy * y + rdx * i + rdy * j + rc

def cast(char):
    y, x = divmod(char, 2)
    return origin + cdx * x + cdy * y + cstc

def cancel(char):
    y, x = divmod(char, 2)
    return origin + cdx * x + cdy * y + cncc

def rclick(pt):
    pag.moveTo(pt)
    pag.mouseDown(pt, button=pag.RIGHT)
    pag.mouseUp(pt, button=pag.RIGHT)

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
