# Dynamic Exploration Planner (DEP) for Robot Exploration
This repo contains the implementation of Dynamic Environment Planner which aims at robotic exploration in dynamic and unknown environments in 
[ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/).

The related paper can be found on: 
**Zhefan Xu, Di Deng, and Kenji Shimada, “Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap,” 
IEEE Robotics and Automation Letters (RA-L), 2021.**

Paper Link (Preprint): https://ieeexplore.ieee.org/document/9362184.

# Installation
This package has been tested on Ubuntu 16.04/18.04 LTS with ROS [Kinetic](http://wiki.ros.org/kinetic)/[Melodic](http://wiki.ros.org/melodic). Make sure you have
installed the compatabile ROS version.

### Simulation Environemnts
To run the planner in simulation, please install the [drone_gazebo](https://github.com/Zhefan-Xu/drone_gazebo).
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/drone_gazebo.git

# Build
cd ~/catkin_ws
catkin_make
```
To visualize the environment, please run the script below:
```
# Use cafe environemnt as an expample. Replace the name of the launch file to change simulation environments.
roslaunch drone_gazebo cafe.launch  
```

### Planner
