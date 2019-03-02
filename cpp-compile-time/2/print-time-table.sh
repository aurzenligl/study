#!/bin/bash

logdir=$1

function die() {
    echo "Usage: ./print-time-table.sh [time-logs-dir]"
    exit 1
}

[ "$#" -eq 1 ] && [ ! -d $logdir ] && die
[ "$#" -ge 2 ] && die
[ -z $logdir ] && logdir='.'

function times {
    echo $1 $2
}

function name {
    abspath=$(echo "$@" | tr ' ' '\n' | grep -m1 "cpp$")
    realpath --relative-to=$(pwd) $abspath
}

export -f times
export -f name

find $logdir -name '*.cpp.o.time' -exec head -1 {} \; | xargs -i bash -c 'printf "$(times {}) $(name {})\n"' | sort -rnk1 | column -t -s' '
