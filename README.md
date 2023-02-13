# Suhan Robot Model Tools
```sh
├── assembly_env_description
│   ├── CMakeLists.txt
│   ├── meshes
│   │   ├── gripper2_base.stl
│   │   ├── gripper2_tip.stl
│   │   ├── gripper_base.stl
│   │   ├── gripper_tip_left.stl
│   │   └── gripper_tip_right.stl
│   ├── models
│   │   ├── bottom.stl
│   │   ├── long_part.stl
│   │   ├── middle_part.stl
│   │   ├── short_part.stl
│   │   ├── side_chair_r.stl
│   │   └── side_chair.stl
│   ├── package.xml
│   ├── README.md
│   ├── robots
│   │   ├── assembly_env.urdf.xacro
│   │   ├── dyros_assembly_gripper2.xacro
│   │   ├── dyros_assembly_gripper.urdf.xacro
│   │   └── dyros_assembly_gripper.xacro
│   ├── scripts
│   │   ├── gen_dh_urdf.py
│   │   └── urdf_ready.py
│   └── vrep
│       ├── assembly_env.ttt
│       └── assembly_env_with_camera.ttt
├── assembly_moveit_config
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── chomp_planning.yaml
│   │   ├── fake_controllers.yaml
│   │   ├── joint_limits.yaml
│   │   ├── kinematics.yaml
│   │   ├── ompl_planning.yaml
│   │   ├── panda.srdf
│   │   ├── ros_controllers.yaml
│   │   └── sensors_3d.yaml
│   ├── launch
│   │   ├── chomp_planning_pipeline.launch.xml
│   │   ├── default_warehouse_db.launch
│   │   ├── demo_gazebo.launch
│   │   ├── demo.launch
│   │   ├── fake_moveit_controller_manager.launch.xml
│   │   ├── gazebo.launch
│   │   ├── joystick_control.launch
│   │   ├── move_group.launch
│   │   ├── move_group_right_only.launch
│   │   ├── moveit.rviz
│   │   ├── moveit_rviz.launch
│   │   ├── ompl_planning_pipeline.launch.xml
│   │   ├── panda_moveit_controller_manager.launch.xml
│   │   ├── panda_moveit.launch
│   │   ├── panda_moveit_sensor_manager.launch.xml
│   │   ├── planning_context.launch
│   │   ├── planning_pipeline.launch.xml
│   │   ├── real.launch
│   │   ├── real_right_only.launch
│   │   ├── ros_controllers.launch
│   │   ├── run_benchmark_ompl.launch
│   │   ├── sensor_manager.launch.xml
│   │   ├── setup_assistant.launch
│   │   ├── trajectory_execution.launch.xml
│   │   ├── warehouse.launch
│   │   └── warehouse_settings.launch.xml
│   ├── package.xml
│   └── README.md
├── CMakeLists.txt
├── franka_ros
│   ├── CHANGELOG.md
│   ├── cmake
│   │   └── ClangTools.cmake
│   ├── franka_control
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── default_combined_controllers.yaml
│   │   │   ├── default_controllers.yaml
│   │   │   ├── franka_combined_control_node.yaml
│   │   │   └── franka_control_node.yaml
│   │   ├── franka_controller_plugins.xml
│   │   ├── include
│   │   │   └── franka_control
│   │   │       └── franka_state_controller.h
│   │   ├── launch
│   │   │   ├── franka_combined_control.launch
│   │   │   └── franka_control.launch
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   └── src
│   │       ├── franka_combined_control_node.cpp
│   │       ├── franka_control_node.cpp
│   │       └── franka_state_controller.cpp
│   ├── franka_description
│   │   ├── CMakeLists.txt
│   │   ├── mainpage.dox
│   │   ├── meshes
│   │   │   └── visual
│   │   │       ├── finger.dae
│   │   │       ├── hand.dae
│   │   │       ├── link0.dae
│   │   │       ├── link1.dae
│   │   │       ├── link2.dae
│   │   │       ├── link3.dae
│   │   │       ├── link4.dae
│   │   │       ├── link5.dae
│   │   │       ├── link6.dae
│   │   │       └── link7.dae
│   │   ├── package.xml
│   │   ├── robots
│   │   │   ├── dual_panda_example.urdf.xacro
│   │   │   ├── hand.urdf.xacro
│   │   │   ├── hand.xacro
│   │   │   ├── panda_arm_hand.urdf.xacro
│   │   │   ├── panda_arm.urdf.xacro
│   │   │   └── panda_arm.xacro
│   │   └── rosdoc.yaml
│   ├── franka_example_controllers
│   │   ├── cfg
│   │   │   ├── compliance_param.cfg
│   │   │   ├── desired_mass_param.cfg
│   │   │   └── dual_arm_compliance_param.cfg
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── franka_example_controllers.yaml
│   │   ├── franka_example_controllers_plugin.xml
│   │   ├── include
│   │   │   └── franka_example_controllers
│   │   │       ├── cartesian_impedance_example_controller.h
│   │   │       ├── cartesian_pose_example_controller.h
│   │   │       ├── cartesian_velocity_example_controller.h
│   │   │       ├── dual_arm_cartesian_impedance_example_controller.h
│   │   │       ├── elbow_example_controller.h
│   │   │       ├── force_example_controller.h
│   │   │       ├── joint_impedance_example_controller.h
│   │   │       ├── joint_position_example_controller.h
│   │   │       ├── joint_velocity_example_controller.h
│   │   │       ├── model_example_controller.h
│   │   │       └── pseudo_inversion.h
│   │   ├── launch
│   │   │   ├── cartesian_impedance_example_controller.launch
│   │   │   ├── cartesian_pose_example_controller.launch
│   │   │   ├── cartesian_velocity_example_controller.launch
│   │   │   ├── dual_arm_cartesian_impedance_example_controller.launch
│   │   │   ├── elbow_example_controller.launch
│   │   │   ├── force_example_controller.launch
│   │   │   ├── joint_impedance_example_controller.launch
│   │   │   ├── joint_position_example_controller.launch
│   │   │   ├── joint_velocity_example_controller.launch
│   │   │   ├── model_example_controller.launch
│   │   │   ├── move_to_start.launch
│   │   │   ├── robot.rviz
│   │   │   └── rviz
│   │   │       ├── franka_description_with_marker.rviz
│   │   │       └── franka_dual_description_with_marker.rviz
│   │   ├── mainpage.dox
│   │   ├── msg
│   │   │   └── JointTorqueComparison.msg
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   ├── scripts
│   │   │   ├── dual_arm_interactive_marker.py
│   │   │   ├── interactive_marker.py
│   │   │   └── move_to_start.py
│   │   └── src
│   │       ├── cartesian_impedance_example_controller.cpp
│   │       ├── cartesian_pose_example_controller.cpp
│   │       ├── cartesian_velocity_example_controller.cpp
│   │       ├── dual_arm_cartesian_impedance_example_controller.cpp
│   │       ├── elbow_example_controller.cpp
│   │       ├── force_example_controller.cpp
│   │       ├── joint_impedance_example_controller.cpp
│   │       ├── joint_position_example_controller.cpp
│   │       ├── joint_velocity_example_controller.cpp
│   │       └── model_example_controller.cpp
│   ├── franka_gripper
│   │   ├── action
│   │   │   ├── Grasp.action
│   │   │   ├── Homing.action
│   │   │   ├── Move.action
│   │   │   └── Stop.action
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   └── franka_gripper_node.yaml
│   │   ├── include
│   │   │   └── franka_gripper
│   │   │       └── franka_gripper.h
│   │   ├── launch
│   │   │   └── franka_gripper.launch
│   │   ├── mainpage.dox
│   │   ├── msg
│   │   │   └── GraspEpsilon.msg
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   └── src
│   │       ├── franka_gripper.cpp
│   │       └── franka_gripper_node.cpp
│   ├── franka_hw
│   │   ├── CMakeLists.txt
│   │   ├── franka_combinable_hw_plugin.xml
│   │   ├── include
│   │   │   └── franka_hw
│   │   │       ├── control_mode.h
│   │   │       ├── franka_cartesian_command_interface.h
│   │   │       ├── franka_combinable_hw.h
│   │   │       ├── franka_combined_hw.h
│   │   │       ├── franka_hw.h
│   │   │       ├── franka_model_interface.h
│   │   │       ├── franka_state_interface.h
│   │   │       ├── resource_helpers.h
│   │   │       ├── services.h
│   │   │       └── trigger_rate.h
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   ├── src
│   │   │   ├── control_mode.cpp
│   │   │   ├── franka_combinable_hw.cpp
│   │   │   ├── franka_combined_hw.cpp
│   │   │   ├── franka_hw.cpp
│   │   │   ├── resource_helpers.cpp
│   │   │   ├── services.cpp
│   │   │   └── trigger_rate.cpp
│   │   └── test
│   │       ├── CMakeLists.txt
│   │       ├── config
│   │       │   └── ros_console_settings_for_tests.conf
│   │       ├── franka_combinable_hw_controller_switching_test.cpp
│   │       ├── franka_hw_controller_switching_test.cpp
│   │       ├── franka_hw_interfaces_test.cpp
│   │       ├── launch
│   │       │   └── franka_hw_test.test
│   │       └── main.cpp
│   ├── franka_msgs
│   │   ├── action
│   │   │   └── ErrorRecovery.action
│   │   ├── CMakeLists.txt
│   │   ├── mainpage.dox
│   │   ├── msg
│   │   │   ├── Errors.msg
│   │   │   └── FrankaState.msg
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   └── srv
│   │       ├── SetCartesianImpedance.srv
│   │       ├── SetEEFrame.srv
│   │       ├── SetForceTorqueCollisionBehavior.srv
│   │       ├── SetFullCollisionBehavior.srv
│   │       ├── SetJointImpedance.srv
│   │       ├── SetKFrame.srv
│   │       └── SetLoad.srv
│   ├── franka_ros
│   │   ├── CMakeLists.txt
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   └── rosdoc.yaml
│   ├── franka_visualization
│   │   ├── CMakeLists.txt
│   │   ├── config
│   │   │   ├── gripper_settings.yaml
│   │   │   └── robot_settings.yaml
│   │   ├── launch
│   │   │   ├── franka_visualization.launch
│   │   │   └── franka_visualization.rviz
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   ├── rosdoc.yaml
│   │   └── src
│   │       ├── gripper_joint_state_publisher.cpp
│   │       └── robot_joint_state_publisher.cpp
│   ├── Jenkinsfile
│   ├── LICENSE
│   ├── NOTICE
│   └── README.md
├── package.xml
├── README.md
├── read.txt
├── setup.py
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
└── tree.txt
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
