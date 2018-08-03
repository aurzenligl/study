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

workspace_packages=$(call catkin_topological_order) || exit

while read line; do
    read -r package path <<<$(echo $line)
    canonical_package=$(echo ${package} | tr '_' '-')
    pushd $path > /dev/null

    call "rm -rf debian"
    call "bloom-generate rosdebian"
    sed -i 's/dh $@/dh $@ --parallel/' debian/rules
    call "debuild -rfakeroot -i -us -uc -b -j`nproc`"
    deb=$(readlink -f ../ros-kinetic-${canonical_package}_*.deb)
    debs="${deb} ${debs}"
    call "sudo dpkg -i $deb"

    popd > /dev/null
done < <(echo "$workspace_packages")

# cleanup, it would probably be better to do it via traps
for deb in $debs; do
    package=$(dpkg -I ${deb} | awk -F: '/Package/ {print $2}')
    call "sudo dpkg -r ${package}"
done
