#!/bin/bash

git clone --branch r3.1.1 --depth 1 https://github.com/mongodb/mongo-cxx-driver.git
cd mongo-cxx-driver
ln -s ../debian
debuild -rfakeroot -us -uc -b -j`nproc`
