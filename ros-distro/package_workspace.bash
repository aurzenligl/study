#!/bin/bash

BLU="\e[34m"
RED="\e[31m"
NOC="\e[0m"

error () {
  echo -e $RED"=== ERROR: $1"$NOC >&2
}

call () {
  echo -e $BLU"$@"$NOC >&2
  eval "$@"
  RC=$?
  if [ "$RC" != "0" ]; then
    error "$@ failed with return code $RC."
    exit 99
  fi
}

while read line; do
    read -r package path <<<$(echo $line)
    pushd $path > /dev/null

    call "rm -rf debian"
    call "bloom-generate rosdebian"
    call "fakeroot debian/rules binary"
    deb=$(readlink -f ../ros-kinetic-${package}_*.deb)
    debs="${deb} ${debs}"
    call "sudo dpkg -i $deb"

    popd > /dev/null
done < <(call catkin_topological_order)

for deb in $debs; do
    package=$(dpkg -I ${deb} | awk -F: '/Package/ {print $2}')
    call "sudo dpkg -r ${package}"
done

# TODO: rosdep rules yaml piece remains to be done
# generate rosdep rules in yaml
# add this yaml to /etc
# rosdep update
