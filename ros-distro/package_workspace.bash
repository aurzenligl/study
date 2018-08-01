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

rulefile=$(mktemp -p /tmp/ --suffix .yaml)
trap "rm -f ${rulefile}" EXIT

workspace_packages=$(call catkin_topological_order)

while read line; do
    read -r package path <<<$(echo $line)
    echo "${package}:" >> ${rulefile}
    echo "  ubuntu: [ros-kinetic-${package}]" >> ${rulefile}
done < <(echo "$workspace_packages")

sudo sh -c "echo yaml file://${rulefile} > /etc/ros/rosdep/sources.list.d/90-package-workspace.list"
trap "{ rm -f ${rulefile}; sudo rm -f /etc/ros/rosdep/sources.list.d/90-package-workspace.list; }" EXIT

call "rosdep update"

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
done < <(echo "$workspace_packages")

# cleanup, it would probably be better to do it via traps
for deb in $debs; do
    package=$(dpkg -I ${deb} | awk -F: '/Package/ {print $2}')
    call "sudo dpkg -r ${package}"
done
