#!/bin/bash

# extract paths (original and includes)
input=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp$")
output=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp\.o$")
hinput=$(echo "$output" | sed 's@\(/[^/]*\).cpp.o$@\1.includes.cpp@g')
houtput="${hinput}.o"

# set commands (original and includes)
cmd="$@"
hcmd=$(echo "$@" | sed "s@$(echo ${input} | sed 's/\./\\./g')@${hinput}@g" | sed "s@${output}@${houtput}@g")

# generate includes-only source file
"$@" -H -E -fdirectives-only 2>&1 >/dev/null | grep -Po "^\. \K.*" | sed -e 's/^\(.*\)/#include "\1"/' > $hinput

# compile modules (original and includes)
$cmd -time=${output}.time
$hcmd -time=${houtput}.time
