#!/bin/bash

sudo apt update
sudo apt install -y ros-kinetic-gazebo9-ros-pkgs ros-kinetic-rosbash ros-kinetic-roslaunch
source /opt/ros/kinetic/setup.bash
rosdep update
