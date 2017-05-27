#!/bin/bash

[ "$CXX" ] || CXX=g++

mkdir -p build

$CXX uk.cpp -shared -o build/libuk.so
$CXX osc.cpp -shared -o build/libosc.so -Lbuild -luk
$CXX trs.cpp -shared -o build/libtrs.so -Lbuild -luk

$CXX main.cpp -Wl,-rpath=build -Lbuild -ltrs -losc
echo noasneeded status=$?

$CXX main.cpp -Wl,--as-needed -Wl,-rpath=build -Lbuild -ltrs -losc
echo asneeded status=$?
