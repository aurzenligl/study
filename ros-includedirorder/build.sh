#!/usr/bin/env bash

scriptdir="$( cd "$( dirname "${BASH_SOURCE[0]}"  )" >/dev/null 2>&1 && pwd  )"
cd $scriptdir

ws0/catkin.sh
echo "this_header_should_have_been_overlaid;" > ws0/devel/include/foo_msgs/Foo.h
ws1/catkin.sh
ws2/catkin.sh
