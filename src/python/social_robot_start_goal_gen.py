from __future__ import print_function, division
from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle
from multiprocessing import Process, Lock, Manager
import random

rospy.init_node('suhan2',anonymous=True)
roscpp_init('suhan', [])

dcc = suhan_robot_model_tools_wrapper_cpp.DualChainConstraintsFunctions()
dcc.add_trac_ik_adapter('left_arm', 'Waist_Pitch','LHand_base', '/robot_description')
dcc.add_trac_ik_adapter('right_arm', 'Waist_Pitch','RHand_base', '/robot_description')
dcc.set_names('left_arm', 'right_arm')

z_diff = 0.125 + 0.2 # desired + hand frame offset

chain_pos = np.array([0, 0, z_diff])
chain_quat = np.array([0, 1, 0, 0])

dcc.set_chain(chain_pos, chain_quat)
dcc.set_max_iterations(500)
dcc.set_tolerance(1e-4)

q_start = [-0.9160884177865198, -1.240112284077678, 0.5994158783047601, -0.5632247309354161, -0.8080804623561342, -1.4702653618796, 
           2.060633453342023, -1.071031767461524, 1.2938335184540475, -0.987905225847562, -0.9449282383464661, -1.2039211367083338]

q_goal= [-1.5160884177865198, -1.240112284077678, 0.5994158783047601, -0.5632247309354161, -0.8080804623561342, -1.4702653618796, 
           1.460633453342023, -1.071031767461524, 1.2938335184540475, -0.987905225847562, -0.9449282383464661, -1.2039211367083338]
q_start = np.array(q_start)
q_goal = np.array(q_goal)
r = dcc.project(q_start)
print (r, q_start)
r = dcc.project(q_goal)
print (r, q_goal)