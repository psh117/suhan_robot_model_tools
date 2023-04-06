from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, DualChainConstraintsFunctions6D, DualChainConstraintIK, OrientationConstraintFunctions, OrientationConstrainedIK, PlanningSceneCollisionCheck, isometry_to_vectors, vectors_to_isometry, MultiChainConstraintFunctions, MultiChainConstraintIK
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import numpy as np
from srmt.planning_scene import PlanningScene
from srmt.utils import ros_init

import time

class ConstraintBase(object):
    def __init__(self, name, dim_constraint):
        ros_init(name)

        self.dim_constraint = dim_constraint
        self.constraint = None
        self.epsilon = 1e-3

    def function(self, q):
        if q.dtype != np.double:
            raise Exception("q is not double")
        x = np.zeros([self.dim_constraint])
        # q = q.astype(np.double)
        self.constraint.function(q,x)
        return x
    
    def is_satisfied(self, q):
        if q.dtype != np.double:
            raise Exception("q is not double")
        # q = q.astype(np.double)
        x = self.function(q)
        norm = np.linalg.norm(x)
        if norm < self.epsilon:
            return True
    
        return False
        
    def project(self, q):
        if q.dtype != np.double:
            raise Exception("q is not double")
        # assert(q.dtype == np.double)
        # q = q.astype(np.double)
        r = self.constraint.project(q)
        return r

    def jacobian(self, q):
        if q.dtype != np.double:
            import pdb; pdb.set_trace()
        # q = q.astype(np.double)
        J = np.zeros([self.dim_constraint, len(q)])
        self.constraint.jacobian(q, J)
        return J

    def set_tolerance(self, tol):
        self.constraint.set_tolerance(tol)

    def set_max_iterations(self, max_iter):
        self.constraint.set_max_iterations(max_iter)

    def set_num_finite_diff(self, num_finite_diff):
        self.constraint.set_num_finite_diff(num_finite_diff)

    def sample(self):
        q = np.random.uniform(low=self.lb, high=self.ub)
        return q
    
    def sample_valid(self, validity_fn, timeout=10.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            q = self.sample()
            r = self.project(q)
            if r is False:
                continue
            if validity_fn(q):
                return q
        return False

    def solve_arm_ik(self, arm_name, q0, pos, quat):
        iso = vectors_to_isometry(pos, quat)
        q_out = np.zeros([len(q0)])
        r = self.ik_solvers[arm_name].solve(q0, iso, q_out)
        return r, q_out 
        #  &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution) 

class ConstraintIKBase(object):
    def __init__(self, name, dim_constraint_ik):
        # ros_init(name)

        self.dim_constraint_ik = dim_constraint_ik
        self.constraint_ik = None

    def function_ik(self, q):
        q = q.astype(np.double)
        x = np.zeros([self.dim_constraint_ik])
        self.constraint_ik.function(q,x)
        return x
        
    def project_ik(self, q):
        assert(q.dtype == np.double)
        # q = q.astype(np.double)
        r = self.constraint_ik.project(q)
        return r

    def jacobian_ik(self, q):
        q = q.astype(np.double)
        J = np.zeros([self.dim_constraint_ik, len(q)])
        self.dim_constraint_ik.jacobian(q, J)
        return J

    def update_target(self, x):
        raise NotImplementedError

    def solve_ik(self, q, x):
        # q = q.astype(np.double)
        x = x.astype(np.double)
        self.update_target(x)
        r = self.project_ik(q)
        return r

class DualArmConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, name1='panda_arm_1', name2='panda_arm_2', ee1='panda_1_hand', ee2='panda_2_hand', desc='/robot_description', base='base', max_iter=1000, tol=1e-3, planning_scene_name='/planning_scenes_suhan', **kwargs):
        super(DualArmConstraint, self).__init__(name='DualArmConstraint', dim_constraint=6)
        self.dim_constraint_ik=12

        self.constraint = DualChainConstraintsFunctions6D()
        self.constraint_ik = DualChainConstraintIK()

        self.ik_solver_1 = self.constraint.add_trac_ik_adapter(name1, base, ee1, 0.1, 1e-6, desc)

        chain_pos = np.array([0.0, 0.0, 0.7])
        chain_quat = np.array([1, 0, 0, 0])

        for c in [self.constraint, self.constraint_ik]:
            ik_solver_1 = c.add_trac_ik_adapter(name1, base, ee1, 0.1, 1e-6, desc)
            ik_solver_2 = c.add_trac_ik_adapter(name2, base, ee2, 0.1, 1e-6, desc)
            c.set_names(name1, name2)
            
            c.set_chain(chain_pos, chain_quat)
            c.set_max_iterations(max_iter)
            c.set_tolerance(tol)
            
        self.ik_solver = ik_solver_1
        self.ik_solvers = {name1: ik_solver_1, name2: ik_solver_2}

        lb_single = self.ik_solver.get_lower_bound()
        ub_single = self.ik_solver.get_upper_bound()
        
        self.planning_scene = PlanningScene([name1, name2], [7, 7], base_link=base, **kwargs)

        # TODO: handle different robots
        self.lb = np.tile(lb_single, 2)
        self.ub = np.tile(ub_single, 2)

    def update_target(self, x):
        assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
        x = x.astype(np.double)
        pos, quat = x[:3], x[3:]
        self.constraint_ik.set_target_pose(pos, quat)
        
    def forward_kinematics(self, name, q):
        q = q.astype(np.double)
        iso = self.ik_solvers[name].forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat


class OrientationConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, name, base, ee, axis=0, orientation_offset=np.identity(3),desc='/robot_description', planning_scene=None, planning_scene_name='/planning_scenes_suhan', **kwargs):
        # axis = 0: x, 1: y, 2: z
        super(OrientationConstraint, self).__init__('OrientationConstraint', dim_constraint=2)
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
        q = q.astype(np.double)
        iso = self.ik_solver.forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat

    def update_target(self, x):
        assert len(x) == 3, 'x must be a 3d vector'
        x = x.astype(np.double)
        self.constraint_ik.set_target_position(x)


class MultiChainConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, arm_names, base_link, ee_links, desc='/robot_description', planning_scene=None, planning_scene_name='/planning_scenes_suhan', **kwargs):
        super(MultiChainConstraint, self).__init__('MultiChainConstraint', dim_constraint=6*(len(arm_names)-1))
        self.dim_constraint_ik = 6*len(arm_names)
        self.constraint = MultiChainConstraintFunctions()
        self.constraint_ik = MultiChainConstraintIK()
        
        nv = NameVector()
        for name in arm_names:
            nv.append(name)

        ik_solver_updated = False
        self.ik_solvers = {}
        lb = []
        ub = []
        for c in [self.constraint, self.constraint_ik]:
            for name, ee in zip(arm_names,ee_links):
                ik_solver = c.add_trac_ik_adapter(name, base_link, ee, 0.1, 1e-6, desc)
                if not ik_solver_updated:
                    self.ik_solvers[name] = ik_solver
                    lb.append(ik_solver.get_lower_bound())
                    ub.append(ik_solver.get_upper_bound())

            ik_solver_updated = True

            c.set_max_iterations(1000)
            c.set_tolerance(1e-3)
            c.set_names(nv)
        
        self.lb = np.concatenate(lb, axis=0)
        self.ub = np.concatenate(ub, axis=0)
        
        if planning_scene is None:
            self.planning_scene = PlanningScene(arm_names, [7]*len(arm_names), topic_name=planning_scene_name, **kwargs)
        # self.planning_scene = PlanningScene(names, [7]*len(names), **kwargs)

        # self.lb = self.ik_solvers.get_lower_bound()
        # self.ub = self.ik_solvers.get_upper_bound()
    
    def set_chains(self, chains):
        """setter for chains

        Args:
            chains (list of np.array): [7D (pos, quat) for each chain]
        """
        self.constraint.set_chains(np.concatenate(chains, axis=0))
        self.constraint_ik.set_chains(np.concatenate(chains, axis=0))
        
    def set_chains_from_joints(self, q):
        """setter for chains

        Args:
            q (np.array): initial joint configurations that will be used to compute the chains
        """
        self.constraint.set_chains_from_joints(q)
        self.constraint_ik.set_chains_from_joints(q)

    def forward_kinematics(self, name, q):
        q = q.astype(np.double)
        iso = self.ik_solvers[name].forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat

    def update_target(self, x):
        assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
        x = x.astype(np.double)
        pos, quat = x[:3], x[3:]
        self.constraint_ik.set_target_pose(pos, quat)