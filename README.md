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

To boot up Virtualbox (local terminal):
`vagrant reload`


Login to Virtualbox.

Within Virtualbox:
```
mkdir catkin_ws
cd catkin_ws
git init
git remote add origin https://github.com/YOUR_USERNAME/REST_RMC_2017.git (Note your github username is required for the full path).
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

Open a new terminal:
```
catkin_make
```

If the make succeeds, you should get going by creating a new package via our [tutorial](https://github.com/pascualy/REST_RMC_2017/tree/master/usage).
