
# download and install gazebo
curl -ssL http://get.gazebosim.org | sh

echo "source /usr/share/gazebo/setup.sh
source ~/catkin_ws/src/rover_gazebo/setup.sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
" >> ~/.bashrc

