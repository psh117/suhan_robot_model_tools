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

fk_set = []
failcount = 0

for i in tqdm(range(args.size)):
    r = False
    if rospy.is_shutdown() == True:
        break
    while r == False:
        q_rnd = np.random.uniform(low=lb, high=ub)
        r = pc.is_valid(q_rnd)
        if r == False:
            failcount += 1
    iso = ik_adaptor.forward_kinematics(q_rnd)
    d = isometry_to_vectors(iso)
    pos = d.first
    quat = d.second
    
    q_rnd = q_rnd.astype(np.float32)
    pos = pos.astype(np.float32)
    quat = quat.astype(np.float32)

    res = (q_rnd, pos, quat)
    fk_set.append(res)

print('saving...')
print('selfcol/trial -- {}/{}'.format(failcount,failcount+args.size))
pickle.dump(fk_set, open('q_dataset_panda_seed_{0}.pkl'.format(seed),'wb'))
print('saved. len:', len(fk_set))