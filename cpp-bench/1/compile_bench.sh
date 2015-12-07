#!/bin/bash

g++ -DGOOGLE_BENCHMARK -DNDEBUG -Wall -Werror -std=c++14 -O3 -fno-omit-frame-pointer main.cpp -lbenchmark -lpthread
