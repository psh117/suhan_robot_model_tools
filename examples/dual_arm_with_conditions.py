from srmt.constraints.constraints import MultiChainConstraint
import numpy as np
import rospy
import time
from scipy.spatial.transform import Rotation as R
import os

from math import pi, sin, cos

name = 'panda_dual_arm_test'
seed = 1107
display = False
save_every = 5000
save_dir = 'datasets'
save_top_k = 4

np.random.seed(seed)

# Left: panda_arm_2
# Right: panda_arm_1
constraint = MultiChainConstraint(['panda_arm_2', 'panda_arm_1'], 
                                'base', 
                                ['panda_2_hand_tcp', 
                                'panda_1_hand_tcp'],
                                hand_names=['hand_2', 'hand_1'], 
                                hand_joints=[2, 2], 
                                hand_open = [[0.0325,0.0325],[0.0325,0.0325]], 
                                hand_closed = [[0.0, 0.0], [0.0, 0.0]])
constraint.set_max_iterations(500)
pc = constraint.planning_scene

obj_pose = np.array([0.3, 0.0, 0.75, 0.0, 0.0, 0.0, 1.0])
first_gen = True
def set_constraint(c):
    global first_gen
    d1, d2, theta = c
    l = d1 + 2*d2*cos(theta)
    ly = l * sin(theta)
    lz = l * cos(theta)
    
    # print(l, ly, lz)
    dt = pi - 2 * theta
    # print('ly: {}, lz: {}'.format(ly, lz))
    chain_pos = np.array([0.0, ly, lz])
    # print('chain_pos: {}'.format(chain_pos))
    chain_rot = np.array([[1, 0, 0], [0, cos(dt), -sin(dt)], [0, sin(dt), cos(dt)]])
    chain_quat = R.from_matrix(chain_rot).as_quat()
    # print('chain_quat: {}'.format(chain_quat))

    t1 = np.concatenate([chain_pos, chain_quat])
    constraint.set_chains([t1])
    if not first_gen:
        pc.detach_object('tray', 'panda_2_hand_tcp')
    pc.add_box('tray', [d1 * 3/4, d1, 0.01], obj_pose[:3], obj_pose[3:])    
    
    first_gen = False
    constraint.set_early_stopping(False)
    
    l_obj_z = d2 + d1/2 * cos(theta)
    l_obj_y = d1/2 * sin(theta)
    ee_to_obj_pos = np.array([0.0, l_obj_y, l_obj_z])
    obj_dt = -(pi/2 + theta)
    ee_to_obj_rot = np.array([[1, 0, 0], [0, cos(obj_dt), -sin(obj_dt)], [0, sin(obj_dt), cos(obj_dt)]])
    ee_to_obj_quat = R.from_matrix(ee_to_obj_rot).as_quat()

    constraint.set_grasp_to_object_pose(go_pos=ee_to_obj_pos, go_quat=ee_to_obj_quat)
    q = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4, 0, 0, 0, -pi/2, 0, pi/2, pi/4])
    constraint.solve_ik(q, obj_pose)
    pc.update_joints(q)
    pc.attach_object('tray', 'panda_2_hand_tcp', [])
    return q

q = np.array([0, 0, 0, -pi/2, 0, pi/2, pi/4, 0, 0, 0, -pi/2, 0, pi/2, pi/4])

d1s = np.linspace(0.2, 0.6, 50)
for d1 in d1s:
    q = set_constraint([d1, 0.05, pi/3])
    # constraint.solve_ik(q, obj_pose)
    pc.display(q)
    time.sleep(0.5)
    for _ in range(10):
        timeout = 0.5
        q = constraint.sample_valid(pc.is_valid, timeout=timeout)
        pc.display(q)
        time.sleep(0.3)

thetas = np.linspace(0, pi/2, 50)
for theta in thetas:
    q = set_constraint([0.4, 0.05, theta])
    # constraint.solve_ik(q, obj_pose)
    pc.display(q)
    time.sleep(0.5)


set_constraint([0.4, 0.05, pi/3])
for _ in range(100):
    timeout = 0.5
    q = constraint.sample_valid(pc.is_valid, timeout=timeout)
    pc.display(q)
    time.sleep(1.0)
