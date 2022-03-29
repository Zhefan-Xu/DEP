# Dynamic Exploration Planner (DEP) for Robot Exploration

03/29 Update: The author will reimplement this algorithm in a cleaner and more structured way in the near future.

This repo contains the implementation of Dynamic Environment Planner (DEP) which aims at robotic exploration in dynamic and unknown environments in 
[ROS](https://www.ros.org/) and [Gazebo](http://gazebosim.org/). This work belongs to CMU Computational Engineering & Robotics Lab (CERLAB).

The related paper can be found on: 

**Zhefan Xu, Di Deng, and Kenji Shimada, “Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap,” 
IEEE Robotics and Automation Letters (RA-L), 2021.**

Paper Link: https://ieeexplore.ieee.org/document/9362184.

Video: https://youtu.be/ileyP4DRBjU

![tunnel_faster](https://user-images.githubusercontent.com/55560905/111251586-ee7b6000-85e5-11eb-8992-d834f2475b45.gif)


<img src="https://user-images.githubusercontent.com/55560905/111251884-77929700-85e6-11eb-8c98-3a28ba0d06d5.gif" alt="cafe_faster" width="278" height="180"><img src="https://user-images.githubusercontent.com/55560905/111250037-09000a00-85e3-11eb-9c04-7b81c4badc74.gif" alt="maze_faster" width="270" height="180"><img src="https://user-images.githubusercontent.com/55560905/111252221-03a4be80-85e7-11eb-8fcc-cab48a055426.gif" alt="office_faster" width="278" height="180">


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

# Add Gazebo Model Path
export GAZEBO_MODEL_PATH=path/to/drone_gazebo/models:$GAZEBO_MODEL_PATH
```
To visualize the environment, please run the script below:
```
# Use cafe environemnt as an expample. Replace the name of the launch file to change simulation environments.
roslaunch drone_gazebo cafe.launch  
```

### Planner
This package relies on [octomap_ros](http://wiki.ros.org/octomap), [voxblox_ros](https://voxblox.readthedocs.io/en/latest/pages/Installation.html), and [nlopt](https://nlopt.readthedocs.io/en/latest/).

Install [octomap_ros](http://wiki.ros.org/octomap).
```
# Install octomap_ros
sudo apt-get install ros-kinetic-octomap-ros
```
Install [voxblox_ros](https://voxblox.readthedocs.io/en/latest/pages/Installation.html) (Modified based on the original installation instruction).
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
Then, add ```esdf.launch``` file into the folder ```~/tsdf_ws/src/voxblox/voxblox_ros/launch```, which contains the following contents: 
```
<launch>
    <node name="esdf_node" pkg="voxblox_ros" type="esdf_server" output="screen" args="-alsologtostderr" clear_params="true">
      <remap from="pointcloud" to="/camera/depth/points"/>
      <remap from="esdf_node/esdf_map_out" to="esdf_map" />
      <param name="tsdf_voxel_size" value="0.2" />
      <param name="tsdf_voxels_per_side" value="16" />
      <param name="publish_esdf_map" value="true" />
      <param name="publish_pointclouds" value="true" />
      <param name="use_tf_transforms" value="true" />
      <param name="update_mesh_every_n_sec" value="1.0" />
      <param name="clear_sphere_for_planning" value="true" />
      <param name="world_frame" value="world" />
      <param name="sensor_frame" value="camera_link"/>
    </node>
</launch>
```

Install [nlopt](https://nlopt.readthedocs.io/en/latest/). First, download the latest file (lastest_version.tar.gz) at: https://nlopt.readthedocs.io/en/latest/#download-and-installation. Then unzip the file into **nlopt** folder.
```
cd path/to/nlopt
mkdir build
cd build
cmake ..
make

cd path/to/nlopt/build
sudo make install
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
source ~/tsdf_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# How to Use
First, launch the simulation environment (E.g. Cafe). You need to turn on the pannel for visualization in [Rviz](http://wiki.ros.org/rviz).
```
roslaunch drone_gazebo cafe.launch
roslaunch voxblox_ros esdf.launch # in a seperate terminal
```

Let the robot(drone) get an initial scan:
```
rosrun drone_gazebo cafe_warmup.sh
```

Finally, run the planner:
```
roslaunch DEP exploration.launch
rosrun DEP move_and_rotate.py # in a seperate temrminal
```

# Visualization
To visualize the exploration process, belows are the ros topics you need to add in [Rviz](http://wiki.ros.org/rviz):

```/voxblox_mesh```: The ESDF map generated from voxblox_ros.

```/occupied_vis_array``` (Optional): The voxel map generated from octomap_server.

```/map_vis_array```: This is the incremental PRM mentioned in the paper.

```/path_vis_array```: The generated path from the DEP planner.

# Restricting the Exploration Range
Sometimes, you may want to explore a confined space (defined by a cubic). In order to achieve that, simply modify the ```include/env.h``` file to change the corresponding dimension of the desired space and then run ```catkin_make``` in ```~/catkin_ws```.

# Citation and Reference:
If you find this work useful, please cite the paper:
```
@article{xu2021autonomous,
  title={Autonomous UAV Exploration of Dynamic Environments via Incremental Sampling and Probabilistic Roadmap},
  author={Xu, Zhefan and Deng, Di and Shimada, Kenji},
  journal={IEEE Robotics and Automation Letters},
  year={2021},
  publisher={IEEE}
}

```
