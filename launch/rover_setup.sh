#!/bin/bash

#read in base hostname and ip
read -p "Enter base hostname(default: pascualy): " roverhostname;
roverhostname=${roverhostname:-pascualy}

read -p "Enter base ip(default: 192.168.1.1): " roverip;
roverip=${roverip:-192.168.1.1}


#Set ROS_MASTER_URI of Base station
export ROS_MASTER_URI=http://$basestationhost:11311;
echo "Set basestation ROS_MASTER_URI as http://$basestationhost:11311";
	
roslaunch ~/catkin_ws/launch/rover.launch;








