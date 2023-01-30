from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, DualChainConstraintsFunctions6D, PlanningSceneCollisionCheck
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle
from tqdm import tqdm
import os

print('import')

name = os.path.basename(__file__).split('.py')[0]
print('[node name]:',name)
rospy.init_node(name, anonymous=True)
roscpp_init(name+'cpp', [])

print('ros')
dcc = DualChainConstraintsFunctions6D()
ad = dcc.add_trac_ik_adapter('panda_arm_1', 'base','panda_1_hand', 0.1, 1e-6, '/robot_description')
dcc.add_trac_ik_adapter('panda_arm_2', 'base','panda_2_hand', 0.1, 1e-6, '/robot_description')
dcc.set_names('panda_arm_1', 'panda_arm_2')

print('DCC')
chain_pos = np.array([0.0, 0.0, 0.7])
chain_quat = np.array([1, 0, 0, 0])

dcc.set_chain(chain_pos, chain_quat)
dcc.set_max_iterations(1000)
dcc.set_tolerance(1e-4)

q_dataset = []

print('PC')
names = NameVector()
dofs = IntVector()
names.append('panda_arm_1')
dofs.append(7)
names.append('panda_arm_2')
dofs.append(7)

pc = PlanningSceneCollisionCheck()
pc.set_group_names_and_dofs(names,dofs)

lb_single = ad.get_lower_bound()
ub_single = ad.get_upper_bound()

lb = np.repeat(lb_single, 2)
ub = np.repeat(ub_single, 2)

for i in tqdm(range(100000)): 
    if rospy.is_shutdown():
        break
    while not rospy.is_shutdown():
        q = np.random.uniform(low=lb, high=ub, size=14)
        r = dcc.project(q)
        if r is True:
            if (q < lb).any():
                continue
            if (q > ub).any():
                continue
            pc.update_joints(q)
            rr = pc.is_valid(q)
            if rr is True:
                # pass
                pc.publish_planning_scene_msg()
                q_dataset.append(q)
                break
        # print(r, q)

        # print(rr)
    #     r = pc.is_valid(q)
    #     if r == False:
    #         failcount += 1
    #         pc.print_current_collision_infos()
pickle.dump(q_dataset, open('q_dataset_6d.pkl','wb'),protocol=pickle.HIGHEST_PROTOCOL)
print('write, len:', len(q_dataset))