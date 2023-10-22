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
    if [[ `echo $state | grep 'key\[10\]=down'` ]]; then  # '1'
        echo -n 1
        ./atk-melee.py
    fi
    if [[ `echo $state | grep 'key\[11\]=down'` ]]; then  # '2'
        echo -n 2
        ./atk-shoot.py
    fi
    if [[ `echo $state | grep 'key\[12\]=down'` ]]; then  # '3'
        echo -n 3
        ./cast-air-shock.py
    fi
    if [[ `echo $state | grep 'key\[13\]=down'` ]]; then  # '4'
        echo -n 4
        ./cast-air-lightning-bolt.py
    fi

    if [[ `echo $state | grep 'key\[31\]=down'` ]]; then  # 'I'
        echo -n 4
        ./cast-air-invisibility.py
    fi
    if [[ `echo $state | grep 'key\[34\]=down'` ]]; then  # '['
        echo -n 4
        ./cast-air-enchant-lightning-arrow.py
    fi
    if [[ `echo $state | grep 'key\[35\]=down'` ]]; then  # ']'
        echo -n 4
        ./cast-air-shock-shield.py
    fi
    if [[ `echo $state | grep 'key\[59\]=down'` ]]; then  # '<'
        echo -n 4
        ./cast-spellcraft-darkness.py
    fi
    if [[ `echo $state | grep 'key\[60\]=down'` ]]; then  # '>'
        echo -n 4
        ./cast-spellcraft-light.py
    fi
done
