#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

rm -rf ${DIR}/build ${DIR}/devel ${DIR}/install ${DIR}/src/ros-kinetic-* ${DIR}/src/*/debian ${DIR}/src/*/obj-x86_64-linux-gnu
