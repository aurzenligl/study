from logged import logged

@logged
def ffoo(a):
    pass

@logged
def fbar(a, b, c=2, d=3):
    pass
