from __future__ import division, print_function

from srmt.constraints import OrientationConstraint, DualArmConstraint, MultiChainConstraint
from srmt.planning_scene import PlanningScene
from srmt.utils import ContinuousGraspCandidates

import numpy as np
import copy
import time
from scipy.spatial.transform import Rotation as R
import os
import sys

from math import pi
import tqdm

import multiprocessing as mp


name = 'panda_triple_arm_ver2'
# seed = 1107
seed_range = range(1107, 1107+6)
display = True
save_every = 100
save_dir = '../datasets/{name}'.format(name=name)
save_top_k = 4
dataset_size = 1000000


def generate(seed):
    np.set_printoptions(precision=3, suppress=True, linewidth=200)
    if sys.version_info.major >= 3:
        os.makedirs(save_dir, exist_ok=True)
    else:
        try:
            os.makedirs(save_dir)
        except:
            pass

    file_name = name + '_dataset_{}'.format(seed)
    np.random.seed(seed)

    print('seed: {}'.format(seed))
    cgc = ContinuousGraspCandidates(file_name='ikea_stefan_without_bottom_cont_grasp.yaml')


    constraint = MultiChainConstraint(['panda_left', 'panda_right', 'panda_top'], 
                                    'base', 
                                    ['panda_left_hand_tcp', 
                                    'panda_right_hand_tcp', 
                                    'panda_top_hand_tcp'])

    pc = constraint.planning_scene

    chair_pos = np.array([0.69, 0.44, 1.19])
    chair_quat = np.array([0.9238795, -0.3826834, 0, 0])
    pc.add_mesh('chair','package://assembly_knowledge/models/meshes/ikea_stefan_without_bottom.stl',
                chair_pos, chair_quat) # X-180, Z-45 Euler

    q_init = np.array([-0.12904, 0.173413, -0.390121, -1.30219, 0.0913822, 1.36203, 1.03038, 
                    -1.53953, -1.64972, 2.00178, -2.66883, 0.633282, 3.66834, 0.562251, 
                    -0.790644, -1.40522, 1.81529, -2.61019, -0.242376, 2.49991, 1.26293])

    q_2 = copy.deepcopy(q_init)
    while True:
        p_cg, q_cg = cgc.get_global_grasp(11, 0.5, chair_pos, chair_quat)
        r, q = constraint.solve_arm_ik('panda_top', q_2[14:21], p_cg, q_cg)
        if r is False:
            continue 
        q_2[14:21] = q

        p_cg, q_cg = cgc.get_global_grasp(68, 0.5, chair_pos, chair_quat)
        r, q = constraint.solve_arm_ik('panda_right', q_2[7:14], p_cg, q_cg)
        if r is False:
            continue
        q_2[7:14] = q

        p_cg, q_cg = cgc.get_global_grasp(27, 0.5, chair_pos, chair_quat)
        r, q = constraint.solve_arm_ik('panda_left', q_2[0:7], p_cg, q_cg)
        if r is False:
            continue
        q_2[0:7] = q
        break
    
    print('initial q: ', q_2)
    pc.update_joints(q_2)
    pc.attach_object('chair', 'panda_left_hand',
                    ['panda_left_hand', 
                    'panda_left_leftfinger', 'panda_left_rightfinger', 
                    'panda_right_hand',
                    'panda_right_finger_left_link', 'panda_right_finger_right_link', 
                    'panda_top_hand',
                    'panda_top_finger_left_link', 'panda_top_finger_right_link'])

    constraint.set_chains_from_joints(q_2)

    tq = tqdm.tqdm(total=dataset_size)

    cnt = 0

    col_cnt = 0
    q_dataset = []

    while True:
        q = constraint.sample()
        r = constraint.project(q)

        if r is False:
            continue

        if (q > constraint.ub).any() or (q < constraint.lb).any():
            # print('out of bound')
            continue

        if pc.is_valid(q) is False:
            # print('collision')
            col_cnt += 1
            tq.set_description('s: {seed:4d}, c: {col_cnt:5d}'.format(seed=seed, col_cnt=col_cnt))
            continue

        cnt += 1
        q_dataset.append(q)
        tq.update(1)

        if display:
            pc.display(q)
        
        if cnt % save_every == 0:
            try:
                np.save('{save_dir}/{file_name}_{cnt}.npy'.format(save_dir=save_dir, file_name=file_name, cnt=cnt), np.array(q_dataset))
                if cnt > save_every*save_top_k:
                    os.remove('{save_dir}/{file_name}_{cnt}.npy'.format(save_dir=save_dir, file_name=file_name, cnt=cnt-save_every*save_top_k))
            except:
                print('save failed')


p_list = []
for seed in seed_range:
    p = mp.Process(target=generate, args=(seed,))
    p.start()
    p_list.append(p)

for p in p_list:
    p.join()