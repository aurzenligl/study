#!/bin/bash

[ "$CXX" ] || CXX=g++
[ $HIDDEN  ] && OPTS="-fvisibility=hidden -fvisibility-inlines-hidden"

mkdir -p build

$CXX lib.cpp -fPIC -shared -o build/liblib.so
$CXX $OPTS main.cpp -Wl,-rpath=build -Lbuild -llib
