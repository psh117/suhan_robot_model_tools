from __future__ import print_function, division
from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
import math
import pickle
from multiprocessing import Process, Lock, Manager
import random
from sensor_msgs.msg import JointState

rospy.init_node('suhan2',anonymous=True)
roscpp_init('suhan', [])


joints = ['Waist_Roll', 'Waist_Pitch', 'Head_Yaw', 'Head_Pitch', 'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch',
  'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 'LFinger_1', 'LFinger_1_2', 'LFinger_2', 'LFinger_2_2',
  'LFinger_3', 'LFinger_3_2', 'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw',
  'RWrist_Pitch', 'RWrist_Roll', 'RFinger_1', 'RFinger_1_2', 'RFinger_2', 'RFinger_2_2', 'RFinger_3',
  'RFinger_3_2', 'active_joint_1', 'active_joint_2', 'active_joint_3', 'active_joint_4']

pos = [0.0] * 32

left_start_idx = 4
right_start_idx = 16

pub = rospy.Publisher('/joint_states',JointState, queue_size=1)
msg = JointState()
msg.name = joints
msg.position = pos
msg.velocity = [0.0] * 32
msg.effort = [0.0] * 32

q_dataset = pickle.load(open('q_dataset_0.325_ver2.pkl','rb'))

# while rospy.is_shutdown() is False:
for q in q_dataset:
    if rospy.is_shutdown():
        break
    msg.position[left_start_idx:left_start_idx+6] = q[:6]
    msg.position[right_start_idx:right_start_idx+6] = q[6:]
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    print(q)
    rospy.sleep(0.1)
# print(joints[left_start_idx])
# print(joints[right_start_idx])
# print(len(joints))
# print(pos)