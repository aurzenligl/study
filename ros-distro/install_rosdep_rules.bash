#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

sudo sh -c "echo 'yaml file://${DIR}/rosdep/foo.yaml' > /etc/ros/rosdep/sources.list.d/90-foo.list"
