# Suhan Robot Model Tools
Code for motion planning and planning scene
## Data Structure
Structure what you need to complete after installation
```sh
├── assembly_env_description 
├── assembly_moveit_config
├── franka_ros
├── src
│   ├── collision_checker
│   │   ├── planning_scene_collision_check.cpp
│   │   └── planning_scene_collision_check.h
│   ├── eigen_tools
│   │   ├── eigen_tools.cpp
│   │   ├── eigen_tools.h
│   │   ├── float_tools.h
│   │   └── suhan_benchmark.h
│   ├── python
│   │   ├── assembly
│   │   │   ├── assembly_datagen.py
│   │   │   ├── assembly_display.py
│   │   │   ├── panda_datagen.py
│   │   │   ├── panda_display.py
│   │   │   └── q_dataset_panda_seed_0.pkl
│   │   ├── dual_panda_6d_exp
│   │   │   └── dual_panda_generate_samples_6d.py
│   │   ├── __init__.py
│   │   ├── social_robot_exp
│   │   │   └── social_robot_collision_datagen.py
│   │   └── test.py
│   ├── suhan_robot_model_tools.cpp
│   ├── suhan_robot_model_tools.h
│   ├── suhan_robot_model_tools_warpper.cpp
│   └── trac_ik_adapter
│       ├── trac_ik_adapter.cpp
│       └── trac_ik_adapter.h
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```
## Installation guide
Install dependencies
```sh
1. melodic
sudo apt install ros-melodic-trac-ik ros-melodic-moveit
sudo apt install ros-melodic-combined-robot-hw

2. noetic
sudo apt install ros-noetic-trac-ik ros-noetic-moveit
sudo apt install ros-noetic-combined-robot-hw
sudo apt install ros-noetic-nlopt

pip install tqdm
sudo apt install libnlopt-dev
sudo apt install libnlopt-cxx-dev
sudo apt install python-numpy
```

Install step and running code example
```sh

cd ~/catkin_ws/src/

git clone https://github.com/psh117/assembly_moveit_config
git clone https://github.com/psh117/assembly_env_description
git clone https://github.com/psh117/suhan_robot_model_tools.git

catkin build

source ../devel/setup.bash

roscd assembly_env_description/robots/

roslaunch assembly_moveit_config demo.launch

# quit the launch after the Rviz load

# rosparam set /robot_description "$(xacro assembly_env.urdf.xacro)" 

roscd suhan_robot_model_tools/src/python/

python panda_datagen.py
```

## Code Instruction

```sh
1. collision_checker
   Use for calculating planning scene collision
   
2. eigen_tools
   c++ library for linear algebra and vector calculus
   
3. python
   main function for planning scene
   
4. trac_ik_adapter
   IK solver Library
   
5. suhan_robot_model_tools
```
