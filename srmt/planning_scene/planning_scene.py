from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, PlanningSceneCollisionCheck, isometry_to_vectors
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import copy
import numpy as np
# import math
from srmt.utils.ros_utils import ros_init

# ros_init = False

class PlanningScene():
    def __init__(self, names, dofs, base_frame_id='/base', hand_name=None, hand_joints=[2], hand_open = [[0.0325,0.0325]], hand_closed = [[0.0, 0.0]], topic_name = "/planning_scenes_suhan", q_init = None) -> None:
        
        ros_init('PlanningScene')

        self.pc = PlanningSceneCollisionCheck(topic_name)
        
        self.use_hand = False
        if hand_name is not None:
            self.hand_name = hand_name
            self.use_hand = True
            
        names_vec = NameVector()
        dofs_vec = IntVector()
        # print('planning scene!')
        for name, dof in zip(names, dofs):
            names_vec.append(name)
            dofs_vec.append(dof)

        self.hand_open = np.array(hand_open, dtype=np.double)
        self.hand_closed = np.array(hand_closed, dtype=np.double)
        self.hand_joints = hand_joints

        if hand_name is not None:
            for name, dof in zip(hand_name, hand_joints):
                names_vec.append(name)
                dofs_vec.append(dof)
        
        self.pc.set_group_names_and_dofs(names_vec,dofs_vec)
        if q_init is not None:
            self.display(q_init)
        self.pc.set_frame_id(base_frame_id)
        
        if hand_name is not None:
            self.gripper_open = [True] * len(hand_name)
        # dim = np.array([0.05,0.05,0.4])
        # pos = np.array([0.33244155,-0.3,1.4-0.25-0.1])
        # quat = np.array([0,0,0,1])
        # self.pc.add_box(dim,'handbox',pos,quat)
        # touch_links = NameVector()
        # self.pc.attach_object('handbox','panda_1_hand',touch_links)

    def add_gripper_to_q(self, q):
        q = copy.deepcopy(q)
        if self.use_hand:
            for g in self.gripper_open:
                if g:
                    q = np.concatenate((q, self.hand_open.flatten()))
                else:
                    q = np.concatenate((q, self.hand_closed.flatten()))
        return q

    def update_joints(self, q):
        q = q.astype(np.double)
        q = self.add_gripper_to_q(q)
        self.pc.update_joints(q)

    def display(self, q):
        self.update_joints(q)
        self.pc.publish_planning_scene_msg()

    def is_valid(self, q):
        q = q.astype(np.double)
        q = self.add_gripper_to_q(q)
        return self.pc.is_valid(q)

    def add_box(self, name, dim, pos, quat):
        self.pc.add_box(np.array(dim,dtype=np.double),name,
                        np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_cylinder(self, name, height, radius, pos, quat):
        self.pc.add_cylinder(np.array([height, radius],dtype=np.double), name, 
                             np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_sphere(self, name, radius, pos, quat):
        self.pc.add_sphere(radius, name, 
                           np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_mesh(self, name, mesh_path, pos, quat):
        self.pc.add_mesh_from_file(mesh_path, name, 
                         np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def attach_object(self, object_id, link_name, touch_links=[]):
        _touch_links = NameVector()
        
        for tl in touch_links:
            _touch_links.append(tl)
        
        self.pc.attach_object(object_id, link_name, _touch_links)

    def detach_object(self, object_id, link_name):
        self.pc.detach_object(object_id, link_name)