echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /home/ubuntu/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/ubuntu/catkin_ws/src/rover_gazebo/setup.sh" >> ~/.bashrc
source ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
