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

    raise Exception('princess does not understand the number')

value = sys.argv[1]
print(princess(value))
