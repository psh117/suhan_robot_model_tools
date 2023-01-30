from __future__ import print_function, division
from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import PlanningSceneCollisionCheck, NameVector, IntVector, TRACIKAdapter, isometry_to_vectors
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle
from multiprocessing import Process, Lock, Manager
import random
from math import pi
from tqdm import tqdm
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--size", type=int, default=1000000)

args = parser.parse_args()

seed = args.seed

np.random.seed(seed)

rospy.init_node('suhan2',anonymous=True)
roscpp_init('suhan', [])

"""
const std::string & name, const std::string & base_link, const std::string & tip_link, double max_time, double precision, const std::string& URDF_param
"""
pc = PlanningSceneCollisionCheck()
names = NameVector()
dofs = IntVector()
names.append('panda_arm')
dofs.append(7)
pc.set_group_names_and_dofs(names,dofs)

# dim = np.array([0.1,0.2,0.3])
# pos_box = np.array([0.5,0.0,1.1])
# quat_box = np.array([0,0,0,1])
# pc.add_box(dim, 'test1',pos_box,quat_box)

ik_adaptor = TRACIKAdapter('panda_link0', 'panda_hand', 0.1, 1e-6,  '/robot_description')
q = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4])

r = pc.is_valid(q)
print ('r',r)
iso = ik_adaptor.forward_kinematics(q)
lb = ik_adaptor.get_lower_bound()
ub = ik_adaptor.get_upper_bound()

d = isometry_to_vectors(iso)
pos = d.first
quat = d.second
# print(pos, quat)
# print(lb, ub)

fk_set = []
failcount = 0

while rospy.is_shutdown() is False:
    # r = False
    # while r == False:
    pc.update_joints(q)
    #     r = pc.is_valid(q)
    #     if r == False:
    #         failcount += 1
    #         pc.print_current_collision_infos()
    pc.publish_planning_scene_msg()
    c = raw_input()
    if c == 'q': break
