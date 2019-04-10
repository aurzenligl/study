from .logged import logged

@logged
def ffoo(a):
    pass

@logged
def fbar(a, b, c=2, d=3):
    pass

@logged
def fbaz():
    ffoo(1)
    fbar(1, 2)
    fbar(3, 4)
    ffoo(5)
