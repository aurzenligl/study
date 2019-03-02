#!/bin/bash

objfile=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp\.o$")
"$@" -time=${objfile}.time -fstats 2>${objfile}.fstats
