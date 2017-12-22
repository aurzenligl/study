reg32 = ''.join(reversed(
    ''.join(chr(ord('0')+x) for x in range(10)) +
    ''.join(chr(ord('a')+x) for x in range(22))
))

x = '........nmlkjihgfedcba9876543210'
y = '......nmlkjihgfedcba9876543210..'
z = '..nmlkji..hgfedc..ba9876..543210'
q = '1032547698badcfehglkjinm........'

def swapm(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b3 + b1 + b2 + b0

def swape(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b0 + b2 + b1 + b3

def swapx(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b1 + b2 + b3 + b0

def swap16(x):
    b0, b1, b2, b3 = x[24:], x[16:24], x[8:16], x[:8]
    return b1 + b0 + b3 + b2

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

ops = [
    deal,
    shfl,
    deal,
    shfl,
    lambda x: rotl(1, x),
    lambda x: rotl(2, x),
    lambda x: rotl(3, x),
    swap,
    swap16,
    swape,
    swapm,
    swapx
]

import random

def magic(x, pred):
    nops = random.randint(2, 20)
    story = []
    for _ in range(nops):
        op = random.choice(ops)
        story.append(op)
        x = op(x)
        if pred(x):
            import pprint
            pprint.pprint('eureka!')
            pprint.pprint(x)
            pprint.pprint(story)
            return True
    return False

def search(x, pred):
    while not magic(x, pred):
        pass

# lambda x: rotl(2, n),

'''
// <      ><      ><      ><      ><      ><      ><      ><      >
//                                 vutsrqponmlkjihgfedcba9876543210  _mem4_const
//                                 0123456789abcdefghijklmnopqrstuv  _bitr
//                                                 PONMLKJIHGFEDCBA  _amem2_const
//                                 ABCDEFGHIJKLMNOP................  _bitr
// 0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP................  _itoll(x, y)
// ..0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP..............  >> 2
// ....0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP............  >> 4
// ......0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP..........  >> 6
// ........0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP........  >> 8
// ..........0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP......  >> 10
// ............0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP....  >> 12
// ..............0123456789abcdefghijklmnopqrstuvABCDEFGHIJKLMNOP..  >> 14
//
// 012345..6789ab..cdefgh..ijklmn..opqrst..uvABCD..EFGHIJ..KLMNOP..  result
'''

