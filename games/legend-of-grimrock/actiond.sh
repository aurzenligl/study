#!/usr/bin/env bash

set -e

winid=`xdotool search --onlyvisible --name "Legend of Grimrock"`
while true;
do
    active=`xdotool getactivewindow`
    ctrl=`xinput --query-state 'SIGMACHIP USB Keyboard' | grep -Po 'key\[10\]=\K.*'`
    if [[ $active = $winid && $ctrl = "down" ]]; then
        echo -n 1
        eval `xdotool getmouselocation --shell`
        xdotool \
            mousemove 1530 850 \
            click --repeat 15 --delay 3 3 \
            mousemove 1750 850 \
            click --repeat 15 --delay 3 3 \
            mousemove $X $Y
    fi
done
