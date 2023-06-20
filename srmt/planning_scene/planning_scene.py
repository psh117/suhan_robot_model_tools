from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, PlanningSceneCollisionCheck, isometry_to_vectors
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import rospy
import copy
import numpy as np
# import math
from srmt.utils import ros_init

# ros_init = False

class PlanningScene(object):
    def __init__(self, arm_names, arm_dofs, base_link='/base', hand_names=None, hand_joints=[2], hand_open = [[0.0325,0.0325]], hand_closed = [[0.0, 0.0]], topic_name = "/planning_scenes_suhan", q_init = None, base_q=None, start_index=None, end_index=None):
        
        ros_init('PlanningScene')

        self.pc = PlanningSceneCollisionCheck(topic_name)
        
        self.base_q = base_q
        self.start_index = start_index
        self.end_index = end_index

        self.use_hand = False
        if hand_names is not None:
            self.hand_names = hand_names
            self.use_hand = True
            
        names_vec = NameVector()
        dofs_vec = IntVector()
        # print('planning scene!')
        for name, dof in zip(arm_names, arm_dofs):
            names_vec.append(name)
            dofs_vec.append(dof)

        self.hand_open = np.array(hand_open, dtype=np.double)
        self.hand_closed = np.array(hand_closed, dtype=np.double)
        self.hand_joints = hand_joints

        if hand_names is not None:
            for name, dof in zip(hand_names, hand_joints):
                names_vec.append(name)
                dofs_vec.append(dof)
        
        self.pc.set_group_names_and_dofs(names_vec,dofs_vec)
        if q_init is not None:
            self.display(q_init)
        self.pc.set_frame_id(base_link)
        
        if hand_names is not None:
            self.gripper_open = [True] * len(hand_names)
        # dim = np.array([0.05,0.05,0.4])
        # pos = np.array([0.33244155,-0.3,1.4-0.25-0.1])
        # quat = np.array([0,0,0,1])
        # self.pc.add_box(dim,'handbox',pos,quat)
        # touch_links = NameVector()
        # self.pc.attach_object('handbox','panda_1_hand',touch_links)

    def add_gripper_to_q(self, q):
        q = copy.deepcopy(q)
        if self.use_hand:
            for g, open, closed in zip(self.gripper_open, self.hand_open, self.hand_closed):
                if g:
                    q = np.concatenate((q, open.flatten()))
                else:
                    q = np.concatenate((q, closed.flatten()))
        return q

    def update_joints(self, q):
        q = q.astype(np.double)
        q = self.add_gripper_to_q(q)
        self.pc.update_joints(q)

    def display(self, q=None):
        if q is not None:
            self.update_joints(q)

        self.pc.publish_planning_scene_msg()

    def display_single(self, q=None):
        if self.base_q is not None:
            self.base_q[self.start_index:self.end_index] = q
            q = self.base_q
            
        if q is not None:
            self.update_joints(q)

        self.pc.publish_planning_scene_msg()

    def is_valid(self, q):
        q = q.astype(np.double)

        if self.base_q is not None:
            self.base_q[self.start_index:self.end_index] = q
            q = self.base_q
            
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

    def update_object_pose(self, object_id, pos, quat):
        self.pc.update_object_pose(object_id, np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def print_current_collision_infos(self):
        self.pc.print_current_collision_infos()