#!/bin/bash

#configure xbox controller
joystick=$(ls /dev/input/js*) || echo "Joystick not found. Please connect usb."; # output error if no js found
sudo chmod a+rw $joystick;

#run launch file
roslaunch ~/catkin_ws/launch/base.launch
