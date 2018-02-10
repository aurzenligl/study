from logged import logged

class Foo():
    @logged
    def __init__(self, a=1):
        pass

    @logged
    def foo(self, a):
        pass

    @logged
    def bar(self, a, b, c=2, d=3):
        pass

    @logged
    @staticmethod
    def sfoo(a):
        pass

    @logged
    @classmethod
    def cfoo(cls, a):
        pass
