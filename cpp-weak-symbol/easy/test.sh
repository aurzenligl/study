#!/bin/bash

g++ main.cpp
./a.out
echo "----------"

g++ main.cpp popen_wrapper.cpp -Wl,--wrap=popen
./a.out
echo "----------"

g++ -fPIC popen_wrapper.cpp -shared -o libpopen_wrapper.so -Wl,--wrap=popen
g++ main.cpp -L. -lpopen_wrapper -Wl,--wrap=popen
LD_LIBRARY_PATH=. ./a.out
echo "----------"
