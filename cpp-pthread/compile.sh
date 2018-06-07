#!/bin/bash

# g++ -Wall -Werror -std=c++14 -Og *.cpp

[ "$CXX" ] || CXX=g++

GOLD="-fuse-ld=gold"
#GOLD="" without gold linker it works

mkdir -p build

$CXX -Dodr_EXPORTS -I. -fdiagnostics-color $GOLD -O2 -g -DNDEBUG -fPIC   -Wall -fPIC -Werror -std=gnu++11 -o build/odr.cpp.o -c odr.cpp
$CXX -fPIC -fdiagnostics-color $GOLD -O2 -g -DNDEBUG  -shared -Wl,-soname,libodr.so -o build/libodr.so build/odr.cpp.o  -lpthread

$CXX -I. -fdiagnostics-color $GOLD -O2 -g -DNDEBUG   -Wall -fPIC -Werror -std=gnu++11 -o build/test.cpp.o -c test.cpp
$CXX -fdiagnostics-color $GOLD -O2 -g -DNDEBUG  build/test.cpp.o -rdynamic build/libodr.so -pthread -lpthread -Wl,-rpath,build
