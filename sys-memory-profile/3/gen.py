#!/usr/bin/env python

import os
import sys

assert len(sys.argv) == 3

ni = int(sys.argv[1])
nj = int(sys.argv[2])

try:
    os.mkdir('gen')
except:
    pass

with open('in.gen_y.cpp') as ingen:
    input = ingen.read()

for i in range(ni):
    for j in range(nj):
        istr = '%d_%d' % (i, j)
        with open('gen/gen%s_y.cpp' % istr, 'w') as outgen:
            outgen.write(input.format(index=istr))
