from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle

rospy.init_node('suhan2')
roscpp_init('suhan', [])

dcc = suhan_robot_model_tools_wrapper_cpp.DualChainConstraintsFunctions()
dcc.add_trac_ik_adapter('left_arm', 'Waist_Pitch','LHand_base', '/robot_description')
dcc.add_trac_ik_adapter('right_arm', 'Waist_Pitch','RHand_base', '/robot_description')
dcc.set_names('left_arm', 'right_arm')

chain_pos = np.array([0, 0, 0.1])
chain_quat = np.array([1, 0, 0, 0])

dcc.set_chain(chain_pos, chain_quat)
dcc.set_max_iterations(1000)
dcc.set_tolerance(1e-4)

q_dataset = []

for i in range(100000):
    if rospy.is_shutdown():
        break
    q = np.random.uniform(low=-math.pi, high=math.pi, size=12)
    r = dcc.project(q)
    if r is True:
        q_dataset.append(q)
        pickle.dump(q_dataset, open('q_dataset.pkl','wb'))
    print(r, q)