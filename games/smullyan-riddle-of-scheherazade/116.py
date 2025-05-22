#!/usr/bin/env python

import sys

def princess(value: str):

    # rule one:
    if value.startswith('1') and value.endswith('2'):
        return value[1:-1]

    # rule two: 3 duplicates
    # princess(x) = y  =>  princess('3' + x) = y + y
    if value.startswith('3'):
        return princess(value[1:]) * 2

    # rule three: 4 reverses
    # princess(x) = y  =>  princess('4' + x) = y[::-1]
    if value.startswith('4'):
        return princess(value[1:])[::-1]

    # rule four: 5 lchops
    # princess(x) = y  =>  princess('5' + x) = y[1:], when len(y) >= 2
    if value.startswith('5'):
        y = princess(value[1:])
        if len(y) < 2:
            raise Exception('princess cannot subtract digit from number with less than two digits')
        return y[1:]

    # rule five: 6 prefixes 1, 7 prefixes 2
    # princess(x) = y  =>  princess('6' + x) = '1' + y
    # princess(x) = y  =>  princess('7' + x) = '2' + y
    if value.startswith('6'):
        return '1' + princess(value[1:])
    if value.startswith('7'):
        return '2' + princess(value[1:])

    raise Exception('princess does not understand the number')

value = sys.argv[1]
print(princess(value))

# 116.
# input:  k 1 456 2
# output: 456 1 456 2
# k = 474536

# k1x2 -> x1x2
# 474536 1 333 2 -> 333 1 333 2

# k1k2 -> k1k2
# 474536 1 474536 2 -> 474536 1 474536 2

# 117.
# k1x2 -> x1x2
# x = 3k
# k13k2 -> 3k13k2
# 3k13k2 -> 3k13k2 3k13k2
# 3474536134745362 -> 3474536134745362 3474536134745362

# 118.
# 4k14k2 -> reversed(4k14k2)
# 4474536144745362 -> 2635474416354744
# 447 453 614 474 536 2 - 16 digits
# 4k = 4474536  # can remove 44 from there
# k' = 74536
# k'1k'2 -> reversed(k'1k'2)
# 745361745362 -> 263547163547

# 119.
# 454 - remove one digit from right side
# 454k1454k2 -> 454k1454k
# 45447453614544745362 -> 4544745361454474536

# 120. - y -> 1y2 -> y
# 121. - y -> 41y2 -> reversed(y)
# 122. - 1y2 -> y -> 2y1
