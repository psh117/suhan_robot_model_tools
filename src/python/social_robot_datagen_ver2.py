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

dataset_filename = 'q_dataset_{0}_ver2.pkl'.format(z_diff)
try:
    q_dataset = pickle.load(open(dataset_filename,'rb'))
except:
    q_dataset = []

manager = Manager()
q_dataset = manager.list(q_dataset)
suc = manager.list([0,0])

# q_dataset = []
print('len:',len(q_dataset))

# dataset_lock = Lock()
# count_lock = Lock()

suc_count = 0
fail_count = 0

def count(r, idx):
    global suc_count
    global fail_count
    if r:
        suc_count += 1
    else:
        fail_count += 1

def save_dataset(new_data):
    q_dataset.append(new_data)
    pickle.dump(q_dataset, open(dataset_filename,'wb'))
    print('saved. len:', len(q_dataset))

def try_project(idx, dataset, suc):
    # while rospy.is_shutdown() is False:
    # random.seed(idx+1)
    np.random.seed(idx+117)
    while True:
        if suc[0] > 100000:
            break
        q = np.random.uniform(low=-math.pi, high=math.pi, size=12)
        r = dcc.project(q)
        for i in range(12):
            if q[i] > math.pi:
                q[i] -= math.pi * 2
            elif q[i] < -math.pi:
                q[i] += math.pi * 2
        if r:
            # save_dataset(q)
            q_dataset.append(q)
            suc[0] += 1
        else:
            suc[1] += 1
        count(r, idx)
        print ('[len: {0}] proc. {1}, rate:'.format(len(q_dataset), idx),suc[0]/(suc[0]+suc[1]),'suc:',suc[0],'fail',suc[1])
        # print(q_dataset)

processes = [(Process(target=try_project, args=(i, q_dataset, suc)), i) for i in range(16)]
for t, i in processes:
    t.start()
    print('thread {0} is running'.format(i))

print('waiting for threads')

for t, i in processes:
    t.join()


q_dataset_save = list(q_dataset)
pickle.dump(q_dataset_save, open(dataset_filename,'wb'))


print('saved. len:', len(q_dataset_save))