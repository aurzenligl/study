#!/bin/bash

mkdir -p /tmp/cpp-app-deps

cd /tmp/cpp-app-deps
wget https://github.com/google/benchmark/archive/v1.3.0.zip -O benchmark.zip
unzip benchmark.zip
cd benchmark-1.3.0
mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Release ..
sudo ninja install

cd /tmp/cpp-app-deps
wget https://github.com/google/googletest/archive/release-1.8.0.zip -O googletest.zip
unzip googletest.zip
cd googletest-release-1.8.0
mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Release ..
sudo ninja install
