echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
mkdir ~/catkin_ws
mkdir ~/catkin_ws/src
cd ~/catkin_ws
/opt/ros/kinetic/bin/catkin_make
echo "source /home/"$USER"/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source /home/"$USER"/catkin_ws/src/rover_gazebo/setup.sh" >> ~/.bashrc
cd ~/catkin_ws
source devel/setup.bash
