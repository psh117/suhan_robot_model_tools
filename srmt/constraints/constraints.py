from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, DualChainConstraintsFunctions6D, DualChainConstraintIK, OrientationConstraintFunctions, OrientationConstrainedIK, PlanningSceneCollisionCheck, isometry_to_vectors
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

    def jacobian(self, q):
        J = np.zeros([self.dim_constraint, len(q)])
        self.constraint.jacobian(q, J)
        return J


class ConstraintIKBase():
    def __init__(self, name, dim_constraint_ik) -> None:
        # ros_init(name)

        self.dim_constraint_ik = dim_constraint_ik
        self.constraint_ik = None

    def function_ik(self, q):
        x = np.zeros([self.dim_constraint_ik])
        self.constraint_ik.function(q,x)
        return x
        
    def project_ik(self, q):
        r = self.constraint_ik.project(q)
        return r

    def jacobian_ik(self, q):
        J = np.zeros([self.dim_constraint_ik, len(q)])
        self.dim_constraint_ik.jacobian(q, J)
        return J

    def update_target(self, x):
        raise NotImplementedError

    def solve_ik(self, q, x):
        self.update_target(x)
        r = self.project_ik(q)
        return r

class DualArmConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, name1='panda_arm_1', name2='panda_arm_2', ee1='panda_1_hand', ee2='panda_2_hand', desc='/robot_description', base='base', max_iter=1000, tol=1e-3, planning_scene_name='/planning_scenes_suhan', **kwargs):
        super().__init__(name='DualArmConstraint', dim_constraint=6)
        self.dim_constraint_ik=12

        self.constraint = DualChainConstraintsFunctions6D()
        self.constraint_ik = DualChainConstraintIK()

        self.ik_solver = self.constraint.add_trac_ik_adapter(name1, base, ee1, 0.1, 1e-6, desc)

        chain_pos = np.array([0.0, 0.0, 0.7])
        chain_quat = np.array([1, 0, 0, 0])

        for c in [self.constraint, self.constraint_ik]:
            ik_solver = c.add_trac_ik_adapter(name1, base, ee1, 0.1, 1e-6, desc)
            ik_solver = c.add_trac_ik_adapter(name2, base, ee2, 0.1, 1e-6, desc)
            c.set_names(name1, name2)
            
            c.set_chain(chain_pos, chain_quat)
            c.set_max_iterations(max_iter)
            c.set_tolerance(tol)
            
        self.ik_solver = ik_solver
        lb_single = self.ik_solver.get_lower_bound()
        ub_single = self.ik_solver.get_upper_bound()
        
        self.planning_scene = PlanningScene([name1, name2], [7, 7], base_frame_id=base, **kwargs)

        # TODO: handle different robots
        self.lb = np.tile(lb_single, 2)
        self.ub = np.tile(ub_single, 2)

    def update_target(self, x):
        assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
        pos, quat = x[:3], x[3:]
        self.constraint_ik.set_target_pose(pos, quat)
        
class OrientationConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, name, base, ee, axis=0, orientation_offset=np.identity(3),desc='/robot_description', planning_scene=None, planning_scene_name='/planning_scenes_suhan', **kwargs):
        # axis = 0: x, 1: y, 2: z
        super().__init__('OrientationConstraint', dim_constraint=2)
        # super().__init__('OrientationConstraint', dim_constraint_ik=5)
        self.dim_constraint = 2
        self.dim_constraint_ik = 5
        self.constraint = OrientationConstraintFunctions()
        self.constraint_ik = OrientationConstrainedIK()
        orientation_vector = np.zeros(3)
        orientation_vector[axis] = 1
        for c in [self.constraint, self.constraint_ik]:
            ik_solver = c.add_trac_ik_adapter(name, base, ee, 0.1, 1e-6, desc)
            c.set_name(name)
            c.set_orientation_vector(orientation_vector)
            c.set_orientation_offset(orientation_offset)

        self.ik_solver = ik_solver
        
        if planning_scene is None:
            self.planning_scene = PlanningScene([name], [7], base_frame_id=base, **kwargs)
        # self.planning_scene = PlanningScene([name], [7], **kwargs)

        self.lb = self.ik_solver.get_lower_bound()
        self.ub = self.ik_solver.get_upper_bound()
    
    def forward_kinematics(self, q):
        iso = self.ik_solver.forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat

    def update_target(self, x):
        assert len(x) == 3, 'x must be a 3d vector'
        self.constraint_ik.set_target_position(x)