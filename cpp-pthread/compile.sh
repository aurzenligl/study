#!/bin/bash

OPTIM="-O2 -g -DNDEBUG"
WRN="-Wall -Werror"
STD="-std=c++11"

GOLD="-fuse-ld=gold"
[ "$CXX" ] || CXX="g++"

mkdir -p build

$CXX $OPTIM $WRN -fPIC $STD -o build/odr.cpp.o -c odr.cpp
$CXX $OPTIM -fPIC -shared -o build/libodr.so build/odr.cpp.o $GOLD -lpthread
# without pthread linking it works
# without linking with gold it works
# with clang it works

$CXX $OPTIM $WRN -fPIC $STD -o build/test.cpp.o -c test.cpp
$CXX $OPTIM build/test.cpp.o -rdynamic build/libodr.so -pthread -lpthread -Wl,-rpath,build
