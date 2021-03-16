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
To run the planner in simulation, please install the [drone_gazebo](https://github.com/Zhefan-Xu/drone_gazebo), which requires [octomap_server](http://wiki.ros.org/octomap_server).
```
sudo apt-get install ros-kinetic-octomap-server
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
This package relies on [octomap_ros](http://wiki.ros.org/octomap), [voxblox_ros](https://voxblox.readthedocs.io/en/latest/pages/Installation.html), and [nlopt](https://nlopt.readthedocs.io/en/latest/).

Install [octomap_ros](http://wiki.ros.org/octomap)
```
# Install octomap_ros
sudo apt-get install ros-kinetic-octomap-ros
```
Install [voxblox_ros](https://voxblox.readthedocs.io/en/latest/pages/Installation.html) (Modified based on the original installation instruction)
```
# Install voxblox_ros (Modified based on the original installation instruction)
mkdir -p ~/tsdf_ws/src
cd ~/tsdf_ws

catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd ~/tsdf_ws/src/
git clone https://github.com/ethz-asl/voxblox.git
wstool init . ./voxblox/voxblox_https.rosinstall
wstool update

cd ~/tsdf_ws/src/
catkin build voxblox_ros
```

Install [nlopt](https://nlopt.readthedocs.io/en/latest/). First, download the latest file (lastest_version.tar.gz) at: https://nlopt.readthedocs.io/en/latest/#download-and-installation. Then unzip the file into **nlopt** folder.
```
cd path/to/nlopt
mkdir build
cd build
cmake ..
make
```

After installing all the required packages, we can compile our planner:
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/DEP.git
```

**PLEASE MAKE SURE TO MODIFY THE FOLLOWING CONTENTS IN CMakeList.txt FOR SUCESSFULL COMPILATION**
```
# Replace ALL "/home/zhefan/Desktop/nlopt/build" to "path/to/nlopt/build"
# Replace ALL "/home/zhefan/Desktop/nlopt/build/libnlopt.so" to "path/to/nlopt/build/libnlopt.so"
```

Also, since this package relies on [catkin_make](http://wiki.ros.org/catkin/commands/catkin_make) instead of [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html), we need to a [catkin worksapces overlay](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying).

TO DO THAT, simply **DELETE** the **build** and **devel** folder in your ```~/catkin_ws```

Finally, compile the planner:
```
cd ~/catkin_ws
catkin_make
```

