# REST_RMC_2017

Assume already installed:
* VirtualBox
* Vagrant
* git


This README will help you set up a virtual environment through vagrant.
```
cd vagrant
vagrant up
```

Please wait for the Vagrantfile to finish set up of the virtualmachine (this may take 20-30 min).

To complete boot up of your Virtual Machine (local terminal):
```
vagrant reload
vagrant ssh
sudo adduser (this lets you set up your username, password, and extra info)
```

Close virtualbox so that we can restart it the Virtual Machine. Now call:
`vagrant up`

Within Virtualbox (open a new terminal):
```
mkdir ~/catkin_ws
cd ~/catkin_ws
git init
git remote add origin https://github.com/pascualy/REST_RMC_2017.git
git fetch
git checkout -t origin/master
```

To allow your VM to remember your git credentials:
```
git config --global user.name "First_Name Last_Name"
git config --global user.email "Your_Email@umich.edu"
git config --global credential.helper cache
git config --global credential.helper 'cache --timeout=3600'
```

Setup Bash
`source /opt/ros/kinetic/setup.bash`

Run Make
```
cd catkin_ws/src
catkin_make
```

If the make succeeds, you should get going by creating a new package via our [tutorial](https://github.com/pascualy/REST_RMC_2017/tree/master/usage).


Upgrading VirtualBox to 5.1 and Extension Pack
```
sudo apt remove virtualbox virtualbox-5.0 virtualbox-4.*

sudo sh -c 'echo "deb http://download.virtualbox.org/virtualbox/debian xenial contrib" >> /etc/apt/sources.list.d/virtualbox.list'

wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -

sudo apt update

sudo apt install virtualbox-5.1

Download and install the extension pack from https://www.virtualbox.org/wiki/Downloads



Package Information:

Rover Description: Contains robot description files for Gazebo and Rviz
	Details: rover.xacro defines the rover
			 rover.gazebo defines the world for the simulation
			 Launch file starts rviz with correct .xacro file and starts joint_state_publisher

Rover Gazebo: Defines the world for the gazebo simulation


Rover Navigation:  



```
