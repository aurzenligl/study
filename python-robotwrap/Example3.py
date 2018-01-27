@decorated
def simplefun():
    pass

class Class(object):
    @decorated
    def __init__(self):
        pass

    @decorated
    def method(self, a, b):
        pass

    @decorated
    @staticmethod
    def staticmet(a1, a2=None):
        pass

    @decorated
    @classmethod
    def classmet(cls):
        pass

def foo():
    @decorated
    def funinfun():
        pass

    class InnerClass(object):
        @decorated
        def metinfun(self, c):
            pass

    funinfun()
    InnerClass().metinfun(12)

simplefun()
inst = Class()
inst.method('xyz', True)
Class.staticmet(a1='323', a2=897)
inst.staticmet(300)
inst.classmet()
Class.classmet()
foo()
