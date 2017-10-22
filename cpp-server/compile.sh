#!/bin/bash

THISDIR=$(readlink -f $(dirname $0))
g++-6 -Wall -Werror -std=c++14 -Og $THISDIR/*.cpp -o $THISDIR/a.out
