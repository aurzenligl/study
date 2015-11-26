#!/bin/bash

g++ -DGOOGLE_BENCHMARK -DNDEBUG -Wall -Werror -std=c++14 -O3 main.cpp -lbenchmark -lpthread
