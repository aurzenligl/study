#!/bin/bash

[ $CXX ] || CXX=g++
$CXX -Wall -std=c++03 -g -pedantic -c abug_*.cpp $@
