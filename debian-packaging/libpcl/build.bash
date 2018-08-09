#!/bin/bash

git clone -n https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout b0b7fa4b49e4c9034f0404c8143f614684940f  # some frozen commit near master chosen for no particular reason
ln -s ../debian
debuild -rfakeroot -us -uc -b -j`nproc`
