reg32 = ''.join(reversed(
    ''.join(chr(ord('0')+x) for x in range(10)) +
    ''.join(chr(ord('a')+x) for x in range(22))
))

x = 'vutsrqponmlkjihgfedcba9876543210'
y = '......nmlkjihgfedcba9876543210..'
z = '..nmlkji..hgfedc..ba9876..543210'
q = '1032547698badcfehglkjinm........'
v = '........................76543210'
t = '......10......32......54......76'
qpsk =  'pohg9810rqjiba32tslkdc54vunmfe76'
qam16 = 'rqpojihgba983210vutsnmlkfedc7654'
q1 = '..tsrqpo..lkjihg..dcba98..543210'
q2 = '........tsrqpolkjihgdcba98543210'

def swap(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b0 + b1 + b2 + b3

def swap02(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b3 + b0 + b1 + b2

def swap12(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b3 + b1 + b2 + b0

def swap03(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b0 + b2 + b1 + b3

def swap13(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b1 + b2 + b3 + b0

def swap2(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b1 + b0 + b3 + b2

def swap4(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b2 + b3 + b0 + b1

def rotl(n, x):
    p0 = x[n:]
    p1 = x[:n]
    return p0 + p1

def bitr(x):
    return x[::-1]

def shfl(x, n=1):
    nh = len(x)/2
    if not n:
        return x
    for _ in range(n):
        x = ''.join(''.join(y) for y in zip(x[:nh], x[nh:]))
    return x

def deal(x, n=1):
    if not n:
        return x
    for _ in range(n):
        x = x[::2] + x[1::2]
    return x

def named(name, fun):
    fun.__name__ = name
    return fun

ops = [
    deal,
    deal,
    deal,
    shfl,
    shfl,
    shfl,
    bitr,
    named('rotl1', lambda x: rotl(1, x)),
    named('rotl2', lambda x: rotl(2, x)),
    named('rotl4', lambda x: rotl(4, x)),
    named('rotl8', lambda x: rotl(8, x)),
    named('rotl24', lambda x: rotl(24, x)),
    swap02,
    swap12,
    swap03,
    swap13,
    swap2,
    swap4,
]

import random

def magic(x, pred):
    nops = random.randint(1, 10)
    story = []
    for _ in range(nops):
        op = random.choice(ops)
        story.append(op.__name__)
        x = op(x)
        if pred(x):
            txt = '('.join(reversed(story)) + '(...)' + (len(story)-1) * ')'
            print('found %s %s' % (x, txt))
            return txt
    return None

def search(x, pred):
    while True:
        out = magic(x, pred)
        if out:
            return out

def searchn(x, pred, n):
    import multiprocessing
    from pathos.multiprocessing import Pool
    nworkers = int(multiprocessing.cpu_count() * 0.75)
    pool = Pool(processes=nworkers)
    def searcher(args):
        return search(*args)
    return pool.map(searcher, [(x, pred)] * n)
