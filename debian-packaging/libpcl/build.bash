#!/bin/bash

git clone --branch pcl-1.8.1 --depth 1 https://github.com/PointCloudLibrary/pcl.git
cd pcl
ln -s ../debian
debuild -rfakeroot -us -uc -b -j`nproc`
