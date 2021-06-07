from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy

rospy.init_node('suhan2')
roscpp_init('suhan', [])

dcc = suhan_robot_model_tools_wrapper_cpp.DualChainConstraintsFunctions()
dcc.add_trac_ik_adapter('left_arm', 'Waist_Pitch','LHand_base', '/robot_description')
dcc.add_trac_ik_adapter('right_arm', 'Waist_Pitch','RHand_base', '/robot_description')
dcc.set_names('left_arm', 'right_arm')