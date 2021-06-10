import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('social_move_group_python_interface',
                anonymous=True)


group_name = 'dual_arm'
group = moveit_commander.MoveGroupCommander(group_name)

joints = ['Waist_Roll', 'Waist_Pitch',
          'LShoulder_Pitch', 'LShoulder_Roll', 'LElbow_Pitch', 'LElbow_Yaw', 'LWrist_Pitch', 'LWrist_Roll', 
          'RShoulder_Pitch', 'RShoulder_Roll', 'RElbow_Pitch', 'RElbow_Yaw', 'RWrist_Pitch', 'RWrist_Roll']

joint_goal = group.get_current_joint_values()

print(joint_goal)
print(len(joint_goal))
jpos = [0.26777388556264337, -0.05656418616259468,
        -2.7450202152680605, -1.5494634997481156, 1.1564279923600238, 0.7353444085524415, 2.2138626624038813, -0.36890717744680673,
        -2.9375591667433643, -0.7703187985386466, -1.950996903879624, -0.29827914411723255, 2.522420883750813, -0.2977575816629342]

q = group.plan(jpos)
print(q)