'''
Allows to capture class of (static|class)method at access and call time.
'''
class decorated(object):
    def __init__(self, func, type_=None):
        self.func = func
        self.type = type_

    '''
    You may inspect class when method is accessed,
    immediately before call or during special-purpose introspection.
    '''
    def __get__(self, obj, type_=None):
        func = self.func.__get__(obj, type_)
        print('accessed %s.%s' % (type_.__name__, func.__name__))
        return self.__class__(func, type_)

    '''
    You may access class at call time.
    '''
    def __call__(self, *args, **kwargs):
        name = '%s.%s' % (self.type.__name__, self.func.__name__)
        print('called %s with args=%s kwargs=%s' % (name, args, kwargs))
        return self.func(*args, **kwargs)

class Foo(object):
    @decorated
    def foo(self, a, b):
        pass

    @decorated
    @staticmethod
    def bar(a, b):
        pass

    @decorated
    @classmethod
    def baz(cls, a, b):
        pass

class Bar(Foo):
    pass

print("Let's try to access things:")
Foo.foo
Foo.bar
Foo.baz
Bar.foo
Bar.bar
Bar.baz
print('')

print("Let's try to call things:")
Foo().foo(1, 2)
Foo.bar(1, b='bcd')
Bar.baz(a='abc', b='bcd')
print('')
