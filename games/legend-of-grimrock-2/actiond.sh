#!/usr/bin/env bash

set -e

winid=`xdotool search --onlyvisible --name "Legend of Grimrock 2" | head -1`
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
        ./cast-fire.py fireburst
        # ./atk-shoot.py
        # ./cast-ice-ice-shards.py
    fi
    if [[ `echo $state | grep 'key\[12\]=down'` ]]; then  # '3'
        echo -n 3
        # ./cast-air-shock.py
        # ./cast-ice-frostbolt.py
        # ./cast-ice-ice-shards.py
    fi
    if [[ `echo $state | grep 'key\[13\]=down'` ]]; then  # '4'
        echo -n 4
        # ./cast-fire-fireball.py
        # ./cast-ice-frostbolt.py
        # ./cast-air-lightning-bolt.py
    fi
done
