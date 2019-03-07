#!/bin/bash

set -e

export CCACHE_DISABLE=1

# extract paths (original and includes)
srcf=$(readlink -f $(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp$"))
base=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp\.o$" | grep -Po ".*(?=\.o)")
input=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp$")
output=$(echo "$@" | tr ' ' '\n' | grep -m1 "\.cpp\.o$")
hinput=$(echo "$output" | sed 's@\(/[^/]*\).cpp.o$@\1.includes.cpp@g')
houtput="${hinput}.o"
hhinput=$(echo "$output" | sed 's@\(/[^/]*\).cpp.o$@\1.header.cpp@g')
hhoutput="${hhinput}.o"

# set commands (original and includes)
cmd="$@"
hcmd=$(echo "$@" | sed "s@$(echo ${input} | sed 's/\./\\./g')@${hinput}@g" | sed "s@${output}@${houtput}@g")
hhcmd=$(echo "$@" | sed "s@$(echo ${input} | sed 's/\./\\./g')@${hhinput}@g" | sed "s@${output}@${hhoutput}@g")

# generate cmd file
echo $cmd > ${base}.cmd
ln -sf $(realpath --relative-to=$(dirname $base) $srcf) $base

# generate includes-only source file
"$@" -H -E -fdirectives-only 2>&1 >/dev/null | grep -Po "^\. \K.*" | xargs -i readlink -f {} | sed -e 's/^\(.*\)/#include "\1"/' > $hinput
head -1 $hinput | grep "/$(basename $input | cut -f 1 -d '.')\.h[a-z]*\"$" > $hhinput || true

# compile modules (original and includes)
$(which time) -f "%U" -o ${hhoutput}.time $hhcmd -Wno-error
$(which time) -f "%U" -o ${houtput}.time $hcmd -Wno-error
$(which time) -f "%U" -o ${output}.time $cmd -Wno-error

# echo $base
# echo $output
# echo $houtput
# echo $hhoutput
