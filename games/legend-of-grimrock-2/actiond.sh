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
        ./cast-water.py frostbolt
        # ./cast-fire.py fireburst
    fi
    if [[ `echo $state | grep 'key\[12\]=down'` ]]; then  # '3'
        echo -n 3
        ./cast-fire.py meteor-storm
        # ./cast-air.py shock
        # ./cast-water.py frostbolt
    fi
    if [[ `echo $state | grep 'key\[13\]=down'` ]]; then  # '4'
        echo -n 4
        ./cast-air.py lightning-bolt
    fi
    if [[ `echo $state | grep 'key\[14\]=down'` ]]; then  # '5'
        echo -n 5
        # ./cast-water.py ice-shards
    fi
    if [[ `echo $state | grep 'key\[52\]=down'` ]]; then  # 'Z'
        echo -n z
        # ./cast-water.py dispel
    fi
    if [[ `echo $state | grep 'key\[53\]=down'` ]]; then  # 'X'
        echo -n x
        # ./cast-fire.py fireball
    fi
    if [[ `echo $state | grep 'key\[54\]=down'` ]]; then  # 'C'
        echo -n c
        # ./cast-water.py dispel
    fi
    if [[ `echo $state | grep 'key\[55\]=down'` ]]; then  # 'V'
        echo -n
        ./cast-water.py dispel
    fi
    if [[ `echo $state | grep 'key\[46\]=down'` ]]; then  # 'L'
        echo -n 'l'
        ./cast-utility.py light
    fi
    if [[ `echo $state | grep 'key\[41\]=down'` ]]; then  # 'F'
        echo -n 'f'
        ./cast-utility.py force-field
    fi
done
