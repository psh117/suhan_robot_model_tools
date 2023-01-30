from __future__ import print_function, division
from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle
from multiprocessing import Process, Lock, Manager
import random
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class StateValidity():
    def __init__(self):
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        rospy.loginfo('service is avaiable')
        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name = []
        self.rs.joint_state.position = []
        self.joint_states_received = False
        self.group_name = ''

    def set_group_name(self, name):
        self.group_name = name

    def set_joint_names(self, names):
        self.rs.joint_state.name = names

    def set_joints(self, joints):
        self.rs.joint_state.position = joints

    def is_valid(self, joints):
        self.set_joints(joints)
        # self.rs.joint_state.position = joints

        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.rs
        gsvr.group_name = self.group_name
        result = self.sv_srv.call(gsvr)
        # print(result)
        return result.valid

# name: ['Waist_Roll', 'Waist_Pitch', Head_Yaw, Head_Pitch, LShoulder_Pitch, LShoulder_Roll, LElbow_Pitch,
#   LElbow_Yaw, LWrist_Pitch, LWrist_Roll, LFinger_1, LFinger_1_2, LFinger_2, LFinger_2_2,
#   LFinger_3, LFinger_3_2, RShoulder_Pitch, RShoulder_Roll, RElbow_Pitch, RElbow_Yaw,
#   RWrist_Pitch, RWrist_Roll, RFinger_1, RFinger_1_2, RFinger_2, RFinger_2_2, RFinger_3,
#   RFinger_3_2, active_joint_1, active_joint_2, active_joint_3, active_joint_4]
# position: [, 0.0, 0.0,  -0.2796, -0.5592, -0.2796, -0.5592, -0.3106, -0.6212, , -0.2796, -0.5592, -0.2796, -0.5592, 0.3106, 0.6212, 0.0, 0.0, 0.0, 0.0]
# velocity: []
# effort: []

joints = ['LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 
          'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

# joints = ['Waist_Roll', 'Waist_Pitch',
#           'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 
#           'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

# jpos = [0.26777388556264337, -0.05656418616259468,
#         -2.7450202152680605, -1.5494634997481156, 1.1564279923600238, 0.7353444085524415, 2.2138626624038813, -0.36890717744680673,
#         -2.9375591667433643, -0.7703187985386466, -1.950996903879624, -0.29827914411723255, 2.522420883750813, -0.2977575816629342]
jpos = [-2.7450202152680605, -1.5494634997481156, 1.1564279923600238, 0.7353444085524415, 2.2138626624038813, -0.36890717744680673,
        -2.9375591667433643, -0.7703187985386466, -1.950996903879624, -0.29827914411723255, 2.522420883750813, -0.2977575816629342]
rospy.init_node('suhan2',anonymous=True)
# roscpp_init('suhan', [])

sv = StateValidity()
sv.set_group_name('dual_arm')
sv.set_joint_names(joints)

print(sv.is_valid(jpos))
fdaf



# joints = ['Waist_Roll', 'Waist_Pitch', 'Head_Yaw', 'Head_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch',
#   'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 'LFinger_1', 'LFinger_1_2', 'LFinger_2', 'LFinger_2_2',
#   'LFinger_3', 'LFinger_3_2', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw',
#   'RWrist_Pitch', 'RWrist_Roll', 'RFinger_1', 'RFinger_1_2', 'RFinger_2', 'RFinger_2_2', 'RFinger_3',
#   'RFinger_3_2', 'active_joint_1', 'active_joint_2', 'active_joint_3', 'active_joint_4']

# pos = [0.0] * 32

# left_start_idx = 4
# right_start_idx = 16

# pub = rospy.Publisher('/joint_states',JointState, queue_size=1)
# msg = JointState()
# msg.name = joints
# msg.position = pos
# msg.velocity = [0.0] * 32
# msg.effort = [0.0] * 32

q_dataset = pickle.load(open('q_dataset_0.325_ver2.pkl','rb'))

# while rospy.is_shutdown() is False:
for q in q_dataset:
    if rospy.is_shutdown():
        break
    r = sv.is_valid(list(q))
    if r:
        print(r,q)
        # print(q)
        # rospy.sleep(5.0)
    # msg.position[left_start_idx:left_start_idx+6] = q[:6]
    # msg.position[right_start_idx:right_start_idx+6] = q[6:]
    # msg.header.stamp = rospy.Time.now()
    # pub.publish(msg)
    # print(q)
    # rospy.sleep(0.1)