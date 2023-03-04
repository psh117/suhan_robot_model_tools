from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, DualChainConstraintsFunctions6D, OrientationConstraintFunctions, PlanningSceneCollisionCheck, isometry_to_vectors
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import numpy as np
from srmt.planning_scene.planning_scene import PlanningScene
from srmt.utils.ros_utils import ros_init

import rospy

class ConstraintBase():
    def __init__(self, name, dim_constraint) -> None:
        ros_init(name)

        self.dim_constraint = dim_constraint
        self.constraint = None

    def function(self, q):
        x = np.zeros([self.dim_constraint])
        self.constraint.function(q,x)
        return x
        
    def project(self, q):
        r = self.constraint.project(q)
        return r



class DualArmConstraint(ConstraintBase):
    def __init__(self, name1='panda_arm_1', name2='panda_arm_2', ee1='panda_1_hand', ee2='panda_2_hand', desc='/robot_description', base='base', max_iter=1000, tol=1e-3, planning_scene_name='/planning_scenes_suhan'):
        super.__init__('DualArmConstraint', dim_constraint=6)

        self.constraint = DualChainConstraintsFunctions6D()
        self.ik_solver = self.constraint.add_trac_ik_adapter(name1, base, ee1, 0.1, 1e-6, desc)
        self.constraint.add_trac_ik_adapter(name2, base, ee2, 0.1, 1e-6, desc)
        self.constraint.set_names(name1, name2)

        chain_pos = np.array([0.0, 0.0, 0.7])
        chain_quat = np.array([1, 0, 0, 0])

        self.constraint.set_chain(chain_pos, chain_quat)
        self.constraint.set_max_iterations(max_iter)
        self.constraint.set_tolerance(tol)

        lb_single = self.ik_solver.get_lower_bound()
        ub_single = self.ik_solver.get_upper_bound()
        
        self.planning_scene = PlanningScene([name1, name2], [7, 7], planning_scene_name)

        self.lb = np.tile(lb_single, 2)
        self.ub = np.tile(ub_single, 2)
        
class OrientationConstraint(ConstraintBase):
    def __init__(self, name, base, ee, axis=0, orientation_offset=np.identity(3),desc='/robot_description', planning_scene_name='/planning_scenes_suhan'):
        # axis = 0: x, 1: y, 2: z
        super().__init__('OrientationConstraint', dim_constraint=2)
        self.constraint = OrientationConstraintFunctions()
        self.ik_solver = self.constraint.add_trac_ik_adapter(name, base, ee, 0.1, 1e-6, desc)
        self.constraint.set_name(name)
        orientation_vector = np.zeros(3)
        orientation_vector[axis] = 1
        self.constraint.set_orientation_vector(orientation_vector)
        self.constraint.set_orientation_offset(orientation_offset)

        self.planning_scene = PlanningScene([name], [7], planning_scene_name)

        self.lb = self.ik_solver.get_lower_bound()
        self.ub = self.ik_solver.get_upper_bound()
    
    def forward_kinematics(self, q):
        iso = self.ik_solver.forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat