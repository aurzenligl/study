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
anri = 'pohg9810rqjiba32tslkdc54vunmfe76'

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

def swaph(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b1 + b0 + b3 + b2

def swapb(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b2 + b3 + b0 + b1

def swap(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b0 + b1 + b2 + b3

def rotl(n, x):
    p0 = x[n:]
    p1 = x[:n]
    return p0 + p1

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
    named('rotl1', lambda x: rotl(1, x)),
    named('rotl2', lambda x: rotl(2, x)),
    named('rotl3', lambda x: rotl(3, x)),
    swap02,
    swap12,
    swap03,
    swap13,
    swaph,
    swapb,
    swap,
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
