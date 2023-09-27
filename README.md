# Suhan Robot Model Tools
Robot model tools for motion planning

## Features
- Collision checker (MoveIt planning scene)
- IK solver (TRAC IK)
- Motion planning (RRT Connect)
- Constraint functions for orientaiton and kinematic constraints
- Visual simulation (OpenGL)
- Python API


## Installation guide
Install dependencies
```sh
sudo apt install ros-$ROS_DISTRO-combined-robot-hw ros-$ROS_DISTRO-trac-ik ros-noetic-moveit ros-$ROS_DISTRO-nlopt
sudo apt install libnlopt-dev libnlopt-cxx-dev libglfw3-dev

pip install tqdm matplotlib
```

Install step and running code example
```sh
cd ~/catkin_ws/src/

git clone https://github.com/psh117/gl_depth_sim
git clone https://github.com/psh117/suhan_robot_model_tools.git

catkin build

source ../devel/setup.bash
```
