from srmt.constraints.constraints import OrientationConstraint
import numpy as np
import rospy
import time
from scipy.spatial.transform import Rotation as R
import os

from math import pi


name = 'panda_arm_orientation_constraint_forward'
seed = 1107
display = False
save_every = 5000
save_dir = 'datasets'
save_top_k = 4


np.random.seed(seed)
"""
input:
q_init = np.array([0,0,0,-pi/2,0,-pi,pi/4])
p,q = oc.forward_kinematics(q_init)
r = R.from_quat(q)
print('r\n',r.as_matrix())
while rospy.is_shutdown() is False:
    oc.planning_scene.display(q=q_init)
    rospy.sleep(0.5)

output: 
r
 [[-2.22044605e-16 -2.22044605e-16  1.00000000e+00]
 [ 2.77555756e-16 -1.00000000e+00 -2.22044605e-16]
 [ 1.00000000e+00  2.77555756e-16  2.22044605e-16]]
"""

R_offset = np.zeros((3,3))
R_offset[2,0] = 1.0
R_offset[1,1] = -1.0
R_offset[0,2] = 1.0

oc = OrientationConstraint('panda_arm', 'panda_link0', 'panda_hand', axis=0, orientation_offset=R_offset, planning_scene_name='/panda_test_scene')

dt = 0.5
last_time = time.time()
positive_sets = []
negative_sets = []
positive_counts = 0
negative_counts = 0
try:
    while rospy.is_shutdown() is False:
        random_q = np.random.uniform(low=oc.lb, high=oc.ub)
        # print(random_q)
        # print(oc.function(random_q))

        r = oc.project(random_q)
        if r is False:
            print('project failed')
            continue
        if (random_q > oc.ub).any() or (random_q < oc.lb).any():
            continue
        if oc.planning_scene.is_valid(random_q) is False:
            continue
        # print(oc.function(random_q))
        # print(random_q)
        p,q = oc.forward_kinematics(random_q)
        r = R.from_quat(q)
        local_r = R_offset @ r.as_matrix()
        if local_r[0,0] > 0.9:
            positive_sets.append(random_q)

            positive_counts = len(positive_sets)
            
            if (positive_counts % save_every) == 0:
                print('positive/negative -- {}/{}'.format(positive_counts,negative_counts))
                try:
                    np.save(f'datasets/{name}_{seed}_positive_{positive_counts}.npy', np.array(positive_sets))
                    if positive_counts > save_every*save_top_k:
                        os.remove(f'datasets/{name}_{seed}_positive_{positive_counts-save_every*save_top_k}.npy')
                except:
                    print('save failed')
            
        elif local_r[0,0] < -0.9:
            negative_sets.append(random_q)
            negative_counts = len(negative_sets)

            if (negative_counts % save_every) == 0:
                print('positive/negative -- {}/{}'.format(positive_counts,negative_counts))
                try:
                    np.save(f'datasets/{name}_{seed}_negative_{negative_counts}.npy', np.array(negative_sets))
                    if negative_counts > save_every*save_top_k:
                        os.remove(f'datasets/{name}_{seed}_negative_{negative_counts-save_every*save_top_k}.npy')
                except:
                    print('save failed')
        else:
            print('what???')
            continue

        if display:
            print('r\n',local_r)
            oc.planning_scene.display(q=random_q)
            rospy.sleep(max(0, dt - (time.time() - last_time)))
            last_time = time.time()

except KeyboardInterrupt:
    pass

print('saving...')
print('positive/negative -- {}/{}'.format(len(positive_sets),len(negative_sets)))
np.save(f'datasets/{name}_{seed}_positive_{len(positive_sets)}.npy', np.array(positive_sets))
np.save(f'datasets/{name}_{seed}_negative_{len(negative_sets)}.npy', np.array(negative_sets))