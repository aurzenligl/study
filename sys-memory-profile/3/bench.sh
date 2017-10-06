#!/bin/bash

valgrind --tool=massif --massif-out-file=massif.out --threshold=0.01 \
./a.out

massif-visualizer massif.out
