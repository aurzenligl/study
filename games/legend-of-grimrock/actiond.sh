#!/usr/bin/env bash

set -e

winid=`xdotool search --onlyvisible --name "Legend of Grimrock"`
while true;
do
    active=`xdotool getactivewindow`
    if [[ $active != $winid ]]; then
        continue
    fi

    state=`xinput --query-state 'SIGMACHIP USB Keyboard'`
    if [[ `echo $state | grep 'key\[10\]=down'` ]]; then
        echo -n 1
        ./atk-melee.py
    fi
    if [[ `echo $state | grep 'key\[11\]=down'` ]]; then
        echo -n 2
        ./cast-shock.py
    fi
done
