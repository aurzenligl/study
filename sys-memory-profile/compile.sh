#!/bin/bash

[ "$1" == "heavy" ] ||
[ "$1" == "light" ] ||
(echo "compile.sh <heavy|light>" && exit)
name=$1

g++ -Wall -Werror -std=c++14 -Og -fvisibility=hidden -fPIC -shared -o liby_$name.so y_$name.cpp &&
g++ -Wall -Werror -std=c++14 -Og x.cpp -ly_$name -L. -Wl,-rpath=. &&
echo "Your buildsystem is beautiful!"
