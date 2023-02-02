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

name = 'panda_assembly_closed_chain_constraint'
tcc = suhan_robot_model_tools_wrapper_cpp.TripleChainConstraintsFunctions()
tcc.add_trac_ik_adapter('left_arm', 'base','panda_left_hand', 0.1, 1e-6,  '/robot_description')
tcc.add_trac_ik_adapter('right_arm', 'base','panda_right_hand', 0.1, 1e-6,  '/robot_description')
tcc.add_trac_ik_adapter('top_arm', 'base','panda_top_hand', 0.1, 1e-6,  '/robot_description')
tcc.set_names('left_arm', 'right_arm', 'top_arm')

q_init = np.array([ -0.974319515483986,0.502209761399138,0.405927984735235,-1.62410050562666, 0.00176466876963385, 2.31114958061098,-0.372115841715219,
            -0.868238261722017,0.553539761670335, 1.01975929351465,-1.78394452707266,-0.611703083258935,  2.32864727930383, 0.666091265175776, 
            0.700846673688826, 0.586681050244243,-0.367622704567015, -1.21504380136177,  0.25971726786195,  1.49314527452455, 0.664837089101801])
z_diff = 0.125 + 0.2 # desired + hand frame offset

chain_pos = np.array([0, 0, z_diff])
chain_quat = np.array([0, 1, 0, 0])

tcc.set_chain(q_init[:7], q_init[7:14], q_init[14:])
tcc.set_max_iterations(4000)
tcc.set_tolerance(1e-3)
tcc.set_num_finite_diff(3) #3, 5, 7
val = np.array([0.0])
tcc.function(q_init,val)
print('val',val)


dataset_filename = 'q_dataset_{0}_ver2.pkl'.format(name)
try:
    q_dataset = pickle.load(open(dataset_filename,'rb'))
except:
    q_dataset = []

manager = Manager()
q_dataset = manager.list(q_dataset)
suc = manager.list([0,0])

# q_dataset = []
print('len:',len(q_dataset))
# print(q_dataset)
# print(len(q_dataset[0]))
# dataset_lock = Lock()
# count_lock = Lock()

suc_count = 0
fail_count = 0

seed_idx = int(random.random() * 1000000)
print('random seed',seed_idx)


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
    np.random.seed(idx+seed_idx)
    while rospy.is_shutdown() is False:
        if suc[0] > 1000000:
            break
        q = np.random.uniform(low=-math.pi, high=math.pi, size=21)
        r = tcc.project(q)
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