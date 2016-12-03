export GAZEBO_MASTER_URI=http://localhost:11345
#export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/rover_gazebo/plugins/EncoderPlugin/build:~/catkin_ws/src/rover_gazebo/plugins/OdometryPlugin/build:~/catkin_ws/src/rover_gazebo/plugins/ControlsPlugin/build
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/rover_gazebo/models
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}
#export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugin_tutorial/build
