import time

class poorprof(object):
    '''Use this object as context manager for profiling.

    It's supposed to profile a single thread of execution,
    hence the use of static data.
    '''
    stack = []
    deadend = None

    def __init__(self,name):
        self.name = name
        self.count = 0

    def __enter__(self):
        if poorprof.stack:
            start = poorprof.stack[-1]._intermittent()
        else:
            start = time.time()
        self.rstart = self.start = start
        poorprof.stack.append(self)

        if poorprof.deadend:
            self._print_report('--', start-poorprof.deadend)
            poorprof.deadend = None

    def __exit__(self, ty, val, tb):
        end = self._report(exit=True)
        assert poorprof.stack.pop() is self
        if poorprof.stack:
            poorprof.stack[-1].start = end
        else:
            poorprof.deadend = end
        return False

    def _intermittent(self):
        self.start = end = self._report()
        return end

    def _report(self, exit=False):
        is_intermittent = not exit or self.count
        is_final = is_intermittent and exit

        end = time.time()
        part = end - self.start
        whole = end - self.rstart if is_final else None
        name = self.name + '.' + chr(ord('a') + self.count) if is_intermittent else self.name
        self.count += 1

        self._print_report(name, part, whole)

        return end

    @staticmethod
    def _print_report(name, part, whole=None):
        spart = '%.3f' % part if part else ''
        swhole = '%.3f' % whole if whole else ''
        indent = (len(poorprof.stack) - 1) * 4 * ' '
        print('pp: %6s %6s %s%s' % (spart, swhole, indent, name))
