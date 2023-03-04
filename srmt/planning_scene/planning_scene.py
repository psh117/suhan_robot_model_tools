from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, PlanningSceneCollisionCheck, isometry_to_vectors
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import numpy as np
# import math
from srmt.utils.ros_utils import ros_init

# ros_init = False

class PlanningScene():
    def __init__(self, names, dofs, topic_name = "/planning_scenes_suhan", q_init = None) -> None:
        
        ros_init('PlanningScene')

        self.pc = PlanningSceneCollisionCheck(topic_name)

        names_vec = NameVector()
        dofs_vec = IntVector()
        # print('planning scene!')
        for name, dof in zip(names, dofs):
            names_vec.append(name)
            dofs_vec.append(dof)
            # print('fdsa',name, dof)
        self.pc.set_group_names_and_dofs(names_vec,dofs_vec)
        if q_init is not None:
            self.pc.update_joints(q_init)
            self.pc.publish_planning_scene_msg()

        # dim = np.array([0.05,0.05,0.4])
        # pos = np.array([0.33244155,-0.3,1.4-0.25-0.1])
        # quat = np.array([0,0,0,1])
        # self.pc.add_box(dim,'handbox',pos,quat)
        # touch_links = NameVector()
        # self.pc.attach_object('handbox','panda_1_hand',touch_links)

    def display(self, q):
        # print('display', q)
        self.pc.update_joints(q)
        self.pc.publish_planning_scene_msg()

    def is_valid(self, q):
        # print(q)
        # self.pc.update_joints(q)
        # self.pc.publish_planning_scene_msg()
        return self.pc.is_valid(q)

    def add_box(self, name, dim, pos, quat):
        self.pc.add_box(np.array(dim,dtype=np.double),name,np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))