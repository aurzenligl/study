#!/bin/bash

name=heavy

g++ -Wall -Werror -std=c++14 -g -O3 -fvisibility=hidden -fPIC -shared -o liby_$name.so y_$name.cpp &&
g++ -Wall -Werror -std=c++14 -g -O3 x.cpp -ly_$name -L. -Wl,-rpath=. &&
echo "Your buildsystem is beautiful!"
