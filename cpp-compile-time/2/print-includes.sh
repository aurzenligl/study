#!/bin/bash

logdir=$1

function die() {
    echo "Usage: ./print-includes.sh [fstats-logs-dir]"
    exit 1
}

[ "$#" -eq 1 ] && [ ! -d $logdir ] && die
[ "$#" -ge 2 ] && die
[ -z $logdir ] && logdir='.'

find $logdir -name '*.cpp.o.fstats' -exec bash -c 'x=$(echo {} | grep -Po "src/\K.*(?=.cpp.o.fstats)"); cat {} | grep -Po "^time in \K/.*(?=:)" | tail -n +2 | sed "s@^@$x -> @g"' \;
