# Final_year_project_MPC

### REQUIREMENTS - ROS, CasADi, Numpy, serial, matplotlib, transformations
Follow ROS installation instructions for Ubuntu in Ros wiki.
```
pip3 install casadi
pip3 install numpy
pip3 install serial
pip3 install matplotlib
pip3 install transformations
```
### Or run requirements.txt
```
pip3 install -r requirements.txt
```

# How to use
```
cd catkin_ws/src
#Clone the  project from github
git clone --recursive https://github.com/nit-ash/Final_year_project_MPC.git
cd ..
catkin_make
```

# Run the project
Give the robot parameters in parameters.py file located in scripts folder.
```
source devel/setup.bash
roslaunch robot_package all_nodes.launch
```
