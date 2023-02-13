# Suhan Robot Model Tools
```bash
|----data
|-------

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
