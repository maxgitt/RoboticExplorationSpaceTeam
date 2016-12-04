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
mkdir catkin_ws
cd catkin_ws
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
