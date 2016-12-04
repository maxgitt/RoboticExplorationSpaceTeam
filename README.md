# REST_RMC_2017

This README will help you set up a virtual environment through vagrant.
```
cd vagrant
vagrant up
```

Please wait for the Vagrantfile to finish set up of the virtualmachine (this may take 20-30 min).







To begin .... from within Virtualbox.
```
mkdir catkin_ws
cd catkin_ws
git init
git remote add origin https://github.com/YOUR_USERNAME/REST_RMC_2017.git (Note your github username is required for the full path).
git fetch
git chekcout -t origin/master
```

To allow your VM to remember your git credentials:
```
git config --global user.name "First_Name Last_Name"
git config --global user.email "Your_Email@umich.edu"
```

Open a terminal:
```
catkin_make
```
