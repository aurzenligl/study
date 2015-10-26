#!/bin/bash

g++ -fPIC -I/usr/include/python2.7 _chi2.c chi2.c -shared -o _chi2.so
