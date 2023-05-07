cmake_minimum_required(VERSION 2.8.3)
project(suhan_robot_model_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_core
  trac_ik_lib
  eigenpy
  pcl_ros
  roslib
  moveit_ros_planning
  moveit_ros_planning_interface
)

SET(OpenGL_GL_PREFERENCE "GLVND")
find_package(gl_depth_sim)

# find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

find_package(ompl REQUIRED)
# find_package(FCL REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(RBDL REQUIRED)

find_package(rostest REQUIRED)

catkin_python_setup()

catkin_package(
  # INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
#  include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${PYTHON_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
#  ${RBDL_INCLUDE_DIR}
 ${OMPL_INCLUDE_DIRS}
 ${gl_depth_sim_INCLUDE_DIRS}
 src
)

add_library(suhan_robot_model_tools
  src/trac_ik_adapter/trac_ik_adapter.cpp
  src/eigen_tools/eigen_tools.cpp
  src/collision_checker/planning_scene_collision_check.cpp
  src/constraints/kinematics_constraint_functions.cpp
  src/constraints/dual_chain_constraint_functions.cpp
  src/constraints/multi_chain_constraint_functions.cpp
  src/constraints/orientation_constraint_functions.cpp
  src/visual_sim/visual_sim.cpp
)

add_library(suhan_robot_model_tools_wrapper_cpp src/suhan_robot_model_tools_warpper.cpp)

target_link_libraries(suhan_robot_model_tools
  ${catkin_LIBRARIES}  
  ${RBDL_LIBRARY}
  ${OMPL_LIBRARIES}
  yaml-cpp
  gl_depth_sim::gl_depth_sim
  gl_depth_sim::glad
  gl_depth_sim::gl_depth_sim_interfaces
)

target_link_libraries(suhan_robot_model_tools_wrapper_cpp 
  suhan_robot_model_tools
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}  
  ${RBDL_LIBRARY}
  ${OMPL_LIBRARIES}
  yaml-cpp
  gl_depth_sim::gl_depth_sim
  gl_depth_sim::glad
  gl_depth_sim::gl_depth_sim_interfaces
)

set_target_properties(suhan_robot_model_tools_wrapper_cpp 
  PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
