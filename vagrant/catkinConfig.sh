echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
/opt/ros/kinetic/bin/catkin_make
echo $ROS_PACKAGE_PATH
echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/ubuntu/catkin_ws/src/rover_gazebo/setup.sh" >> ~/.bashrc
source devel/setup.bash
