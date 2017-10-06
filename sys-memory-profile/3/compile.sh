#!/bin/bash

name=heavy

g++ -Wall -Werror -std=c++14 -g -Os -fvisibility=hidden -fPIC -shared -o liby_$name.so y_$name.cpp gen/*.cpp &&
g++ -Wall -Werror -std=c++14 -g -Os x.cpp -ly_$name -L. -Wl,-rpath=. &&
echo "Your buildsystem is beautiful!"
