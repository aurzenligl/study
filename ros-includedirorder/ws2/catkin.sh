#!/usr/bin/env bash

scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" >/dev/null 2>&1 && pwd  )"
cd $scriptdir

rm -rf build devel
source ../ws1/devel_symlink/setup.bash
catkin_make --use-ninja
