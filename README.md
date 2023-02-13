# Suhan Robot Model Tools
## Data Structure
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

```sh
# install dependencies
sudo apt install ros-melodic-trac-ik ros-melodic-moveit
```


For an example,
```sh

sudo apt install ros-melodic-combined-robot-hw
pip install tqdm

cd ~/catkin_ws/src/

git clone https://github.com/psh117/assembly_moveit_config
git clone https://github.com/psh117/assembly_env_description
git clone -b develop https://github.com/psh117/franka_ros.git

catkin build

source ../devel/setup.bash

roscd assembly_env_description/robots/

roslaunch assembly_moveit_config demo.launch

# quit the launch after the Rviz load

#rosparam set /robot_description "$(xacro assembly_env.urdf.xacro)" 

roscd suhan_robot_model_tools/src/python/

python panda_datagen.py
```
