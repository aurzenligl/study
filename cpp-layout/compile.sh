#!/bin/bash

[ $CXX ] || CXX=g++
$CXX -Wall -std=c++98 -g -pedantic padding.cpp $@
