#!/bin/bash

THISDIR=$(readlink -f $(dirname $0))
g++ -Wall -Werror -std=c++14 -Og $THISDIR/*.cpp -o $THISDIR/a.out
