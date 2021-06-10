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

dcc = suhan_robot_model_tools_wrapper_cpp.DualChainConstraintsFunctions()
dcc.add_trac_ik_adapter('left_arm', 'Waist_Pitch','LHand_base', '/robot_description')
dcc.add_trac_ik_adapter('right_arm', 'Waist_Pitch','RHand_base', '/robot_description')
dcc.set_names('left_arm', 'right_arm')

z_diff = 0.125 + 0.2 # desired + hand frame offset

chain_pos = np.array([0, 0, z_diff])
chain_quat = np.array([0, 1, 0, 0])

dcc.set_chain(chain_pos, chain_quat)
dcc.set_max_iterations(500)
dcc.set_tolerance(1e-2) # real usage okay

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

q_dataset = pickle.load(open('q_out_for_viz_social.pkl','rb'))


diff = np.linalg.norm(np.array(q_dataset[0]) - np.array(q_dataset[1]))
print(diff)
projected_q = []
for q in q_dataset:
  if rospy.is_shutdown():
      exit()
  r = dcc.project(q)
  print (r)
  projected_q.append(q)

print ('reprojection done')

print ('interpolate start/goal')
q_start = [-0.93003348, -1.3897195,0.66266431, -0.6554421,  -0.70950444, -1.38370451,
            2.07457851, -1.20216449,  1.19728489, -0.91721626, -1.10913862, -1.03883084]

q_goal = [-1.53003348, -1.3897195,   0.66266431, -0.6554421,  -0.70950444, -1.38370451,
          1.47457851, -1.20216449,  1.19728489, -0.91721626, -1.10913862, -1.03883084]
# projected_q_aug = [q_start] + projected_q + [q_goal]

q_start, q_goal = np.array(q_start), np.array(q_goal)
start_diff = np.linalg.norm(np.array(q_start) - projected_q[0])
goal_diff = np.linalg.norm(np.array(q_goal) - projected_q[-1])


delta = 0.067

num_iter_points_start = int(start_diff / delta)
num_iter_points_goal = int(goal_diff / delta)

starts = []
start_diff = (projected_q[0] - q_start)
for i in range (1,num_iter_points_start):
  int_start_q = q_start + start_diff * (i/num_iter_points_start)
  dcc.project(int_start_q)
  starts.append(int_start_q)

goals = []
goal_diff = (q_goal - projected_q[-1])
for i in range (1,num_iter_points_goal):
  int_goal_q = projected_q[-1] + goal_diff * (i/num_iter_points_goal)
  dcc.project(int_goal_q)
  goals.append(int_goal_q)

projected_q_aug = [q_start] + starts + projected_q + goals + [q_goal]

print ('interpolate start/goal done')
# while rospy.is_shutdown() is False:
for _ in range(100):
  for q in projected_q_aug:
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