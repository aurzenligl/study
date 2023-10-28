#!/usr/bin/env bash

set -e

winid=`xdotool search --onlyvisible --name "Legend of Grimrock" | head -1`
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
        ./cast-fire-fireburst.py
        # ./atk-shoot.py
        # ./cast-ice-ice-shards.py
    fi
    if [[ `echo $state | grep 'key\[12\]=down'` ]]; then  # '3'
        echo -n 3
        ./cast-air-shock.py
        # ./cast-ice-frostbolt.py
        # ./cast-ice-ice-shards.py
    fi
    if [[ `echo $state | grep 'key\[13\]=down'` ]]; then  # '4'
        echo -n 4
        ./cast-fire-fireball.py
        # ./cast-ice-frostbolt.py
        # ./cast-air-lightning-bolt.py
    fi
    if [[ `echo $state | grep 'key\[14\]=down'` ]]; then  # '5'
        echo -n 5
        ./cast-air-lightning-bolt.py
    fi
    if [[ `echo $state | grep 'key\[18\]=down'` ]]; then  # '9'
        echo -n 9
        ./cast-bitd-weaken-nonmaterial-beings.py
    fi
    if [[ `echo $state | grep 'key\[19\]=down'` ]]; then  # '0'
        echo -n 0
        ./cast-bitd-zo.py
    fi

    if [[ `echo $state | grep 'key\[31\]=down'` ]]; then  # 'I'
        echo -n 'i'
        # ./cast-air-invisibility.py
    fi
    if [[ `echo $state | grep 'key\[34\]=down'` ]]; then  # '['
        echo -n '['
        # ./cast-air-enchant-lightning-arrow.py
    fi
    if [[ `echo $state | grep 'key\[35\]=down'` ]]; then  # ']'
        echo -n ']'
        # ./cast-air-shock-shield.py
    fi
    if [[ `echo $state | grep 'key\[43\]=down'` ]]; then  # 'H'
        echo -n 'h'
        ./cast-bitd-potion-health.py
    fi
    if [[ `echo $state | grep 'key\[58\]=down'` ]]; then  # 'M'
        echo -n 'h'
        ./cast-bitd-potion-energy.py
    fi
    if [[ `echo $state | grep 'key\[46\]=down'` ]]; then  # 'L'
        echo -n 'l'
        ./cast-spellcraft-light.py
    fi
    if [[ `echo $state | grep 'key\[59\]=down'` ]]; then  # '<'
        echo -n '<'
        # ./cast-spellcraft-darkness.py
    fi
    if [[ `echo $state | grep 'key\[60\]=down'` ]]; then  # '>'
        echo -n '>'
        # ./cast-spellcraft-light.py
    fi

    # utility

    if [[ `echo $state | grep 'key\[110\]=down'` ]]; then  # 'Home'
        echo -n '[h1]'
        # ./drink-healing.py 1
    fi
    if [[ `echo $state | grep 'key\[112\]=down'` ]]; then  # 'PageUp'
        echo -n '[h2]'
        # ./drink-healing.py 2
    fi
    if [[ `echo $state | grep 'key\[115\]=down'` ]]; then  # 'End'
        echo -n '[h3]'
        # ./drink-healing.py 3
    fi
    if [[ `echo $state | grep 'key\[117\]=down'` ]]; then  # 'PageDown'
        echo -n '[h4]'
        # ./drink-healing.py 4
    fi
done
