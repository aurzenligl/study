#!/bin/bash

git clone --branch 1.6.3 --depth 1 --recurse-submodules https://github.com/mongodb/mongo-c-driver.git
cd mongo-c-driver
ln -s ../debian
debuild -rfakeroot -us -uc -b -j`nproc`
