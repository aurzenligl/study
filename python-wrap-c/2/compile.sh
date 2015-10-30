#!/bin/bash

g++ -std=c++11 -fPIC -I/usr/include/python2.7 _chi2.cpp chi2.cpp -shared -o _chi2.so
