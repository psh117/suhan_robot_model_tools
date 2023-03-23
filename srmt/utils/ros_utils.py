
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

ros_initialized = False

def ros_init(name):
    global ros_initialized
    if ros_initialized is False:
        rospy.init_node(name, anonymous=True, disable_signals=True)
        roscpp_init('{name}cpp'.format(name=name), [])
        ros_initialized = True