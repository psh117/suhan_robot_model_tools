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

# q = [-0.9160884177865198, -1.240112284077678, 0.5994158783047601, -0.5632247309354161, -0.8080804623561342, -1.4702653618796, 
#            2.060633453342023, -1.071031767461524, 1.2938335184540475, -0.987905225847562, -0.9449282383464661, -1.2039211367083338]
q_dataset = []
q1 = [-0.93003348, -1.3897195,0.66266431, -0.6554421,  -0.70950444, -1.38370451,
     2.07457851, -1.20216449,  1.19728489, -0.91721626, -1.10913862, -1.03883084]
q_dataset.append(q1)
q2 = [-1.53003348, -1.3897195,   0.66266431, -0.6554421,  -0.70950444, -1.38370451,
  1.47457851, -1.20216449,  1.19728489, -0.91721626, -1.10913862, -1.03883084]

q_dataset.append(q2)

q3 = [-0.97515036,  3.01969886, -2.57107943,  2.5057343,   2.03198213,  1.73730074,
  0.37699112, -1.86987595,  1.53435385,  0.10869911,  1.45581404,  2.22236264]
q_dataset.append(q3)
q_dataset = q_dataset * 10
# while rospy.is_shutdown() is False:
for q in q_dataset:
    if rospy.is_shutdown():
        break
    msg.position[left_start_idx:left_start_idx+6] = q[:6]
    msg.position[right_start_idx:right_start_idx+6] = q[6:]
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    # print(q)
    rospy.sleep(1.0)
# print(joints[left_start_idx])
# print(joints[right_start_idx])
# print(len(joints))
# print(pos)