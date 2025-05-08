#!/usr/bin/env python

import sys

def princess(value: str):

    # rule 1:
    if value.startswith('1') and value.endswith('2'):
        return value[1:-1]

    # rule 2:
    # princess(x) = y  =>  princess('3' + x) = y + y
    if value.startswith('3'):
        return princess(value[1:]) * 2

    # rule 3:
    # princess(x) = y  =>  princess('4' + x) = y[::-1]
    if value.startswith('4'):
        return princess(value[1:])[::-1]

    # rule 4:
    # princess(x) = y  =>  princess('5' + x) = y[1:], when len(y) >= 2
    if value.startswith('5'):
        y = princess(value[1:])
        if len(y) < 2:
            raise Exception('princess cannot subtract digit from number with less than two digits')
        return y[1:]

    # rule 5:
    # princess(x) = y  =>  princess('6' + x) = '1' + y
    # princess(x) = y  =>  princess('7' + x) = '2' + y
    if value.startswith('6'):
        return '1' + princess(value[1:])
    if value.startswith('7'):
        return '2' + princess(value[1:])

    raise Exception('princess does not understand the number')

value = sys.argv[1]
print(princess(value))
