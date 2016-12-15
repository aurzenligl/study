#!/bin/bash

[ $CXX ] || CXX=g++
$CXX -Wall -Wextra -std=c++03 -g -pedantic -c abug_*.cpp $@
