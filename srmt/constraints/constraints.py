from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import NameVector, IntVector, DualChainConstraintsFunctions6D, DualChainConstraintIK, OrientationConstraintFunctions, OrientationConstrainedIK, PlanningSceneCollisionCheck, isometry_to_vectors, vectors_to_isometry, MultiChainConstraintFunctions, MultiChainConstraintIK, MultiChainWithFixedOrientationConstraint, ParallelConstraint
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
import numpy as np
from srmt.planning_scene import PlanningScene
from srmt.utils import ros_init
from srmt.utils import get_pose, get_transform
from scipy.spatial.transform import Rotation as R
import time

class ConstraintBase(object):
    def __init__(self, name, dim_constraint):
        ros_init(name)

        self.dim_constraint = dim_constraint
        self.constraint = None
        self.epsilon = 5e-3

        self.lb = None
        self.ub = None

        self.arm_names = []

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

    def set_early_stopping(self, early_stopping):
        self.constraint.set_early_stopping(early_stopping)

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
            
            if self.lb is not None and np.any(q < self.lb):
                continue

            if self.ub is not None and np.any(q > self.ub):
                continue
            
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
        self.constraint_ik.jacobian(q, J)
        return J

    def update_target(self, x):
        raise NotImplementedError

    def get_object_pose(self, q):
        raise NotImplementedError

    def solve_ik(self, q, x):
        # q = q.astype(np.double)
        x = x.astype(np.double)
        self.update_target(x)
        r = self.project_ik(q)
        return r


class DualArmConstraint(ConstraintBase, ConstraintIKBase): # deprecated. use MultiChainConstraint
    def __init__(self, name1='panda_arm_1', name2='panda_arm_2', ee1='panda_1_hand', ee2='panda_2_hand', arm_dofs=[], desc='/robot_description', base='base', max_iter=1000, tol=1e-3, planning_scene_name='/planning_scenes_suhan', **kwargs):
        super(DualArmConstraint, self).__init__(name='DualArmConstraint', dim_constraint=6)
        self.dim_constraint = 6
        self.dim_constraint_ik = 12

        arm_dof = sum(arm_dofs)
        self.constraint = DualChainConstraintsFunctions6D(arm_dof, self.dim_constraint)
        self.constraint_ik = DualChainConstraintIK(arm_dof, self.dim_constraint_ik)

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
            
        self.constraint_ik.set_step_size(0.5)
        self.constraint_ik.set_early_stopping(True)

        self.ik_solver = ik_solver_1
        self.ik_solvers = {name1: ik_solver_1, name2: ik_solver_2}

        lb_single = self.ik_solver.get_lower_bound()
        ub_single = self.ik_solver.get_upper_bound()
        
        self.planning_scene = PlanningScene(arm_names=[name1, name2], arm_dofs=arm_dofs, base_link=base, **kwargs)

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
    def __init__(self, arm_names, base_link, ee_links, arm_dofs=[7], constraint_start_index=None, constraint_end_index=None, axis=0, reversed_axis=False, orientation_offset=np.identity(3),desc='/robot_description', planning_scene=None, **kwargs):
        # axis = 0: x, 1: y, 2: z
        # reversed_axis: if True, the direction is reversed along the given axis
        super(OrientationConstraint, self).__init__('OrientationConstraint', dim_constraint=2)
        self.dim_constraint = 2
        self.dim_constraint_ik = 5

        self.arm_dofs = arm_dofs
        self.orientation_offset = orientation_offset
        self.reversed_axis = reversed_axis
        self.axis = axis
        self.orientation_vector = np.zeros(3)
        self.orientation_vector[axis] = 1

        if constraint_start_index is None:
            constraint_start_index = 0
        
        arm_dof = arm_dofs[constraint_start_index]

        self.constraint = OrientationConstraintFunctions(arm_dof, self.dim_constraint)
        self.constraint_ik = OrientationConstrainedIK(arm_dof, self.dim_constraint_ik)
        for c in [self.constraint, self.constraint_ik]:
            ik_solver = c.add_trac_ik_adapter(arm_names[constraint_start_index], base_link, ee_links[constraint_start_index], 0.1, 1e-6, desc)
            c.set_name(arm_names[constraint_start_index])
            c.set_orientation_vector(self.orientation_vector)
            c.set_orientation_offset(orientation_offset)  

        self.ik_solver = ik_solver
        self.ik_solvers = {arm_names[0]: ik_solver}

        self.arm_names = arm_names
        self.arm_dofs = arm_dofs
        
        if planning_scene is None:
            self.planning_scene = PlanningScene(arm_names=arm_names, arm_dofs=arm_dofs, base_link=base_link, **kwargs)

        self.lb = self.ik_solver.get_lower_bound()
        self.ub = self.ik_solver.get_upper_bound()
    
    def forward_kinematics(self, name, q):
        q = q.astype(np.double)
        iso = self.ik_solvers[name].forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat

    def update_target(self, x):
        assert len(x) == 3, 'x must be a 3d vector'
        x = x.astype(np.double)
        self.constraint_ik.set_target_position(x)

    def project(self, q):
        r = super().project(q)
        if r is False:
            return False
        
        r = self.check_orientation_side(q)
        return r
        
    def check_orientation_side(self, q):
        _, quat = self.forward_kinematics(self.arm_names[0], q)
        r = R.from_quat(quat)
        local_r = self.orientation_offset.T @ r.as_matrix()

        if local_r[self.axis, self.axis] > 0.9:
            # True if not reversed
            return not self.reversed_axis
        
        elif local_r[self.axis, self.axis] < -0.9:
            # True if reversed
            return self.reversed_axis
        
        else:
            print('local_r', local_r)
            print('r', r.as_matrix())
            print('self.orientation_offset', self.orientation_offset)
            raise Exception("Something's wrong")
        
        return False
    
class MultiChainConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, arm_names, base_link, ee_links, arm_dofs=[], 
                 constraint_start_index=None, constraint_end_index=None, 
                 desc='/robot_description', planning_scene=None, 
                 planning_scene_name='/planning_scenes_suhan', 
                 **kwargs):

        self.arm_dofs = arm_dofs
        
        if constraint_end_index is None:
            constraint_end_index = len(arm_names)
        if constraint_start_index is None:
            constraint_start_index = 0

        len_arms = constraint_end_index-constraint_start_index
    
        super(MultiChainConstraint, self).__init__('MultiChainConstraint', dim_constraint=6*(len_arms-1))

        self.dim_constraint = 6*(len_arms-1)
        self.dim_constraint_ik = 6*(len_arms)

        arm_dof = sum(arm_dofs[constraint_start_index:constraint_end_index])

        constraints = []
        self.constraint = MultiChainConstraintFunctions(arm_dof, self.dim_constraint)
        constraints.append(self.constraint)
        
        if arm_dof > self.dim_constraint_ik:
            self.constraint_ik = MultiChainConstraintIK(arm_dof, self.dim_constraint_ik)
            self.constraint_ik.set_step_size(0.1)
            self.constraint_ik.set_early_stopping(True)
            constraints.append(self.constraint_ik)
        else:
            self.constraint_ik = None

        self.arm_names = arm_names[constraint_start_index:constraint_end_index]
        nv = NameVector()
        self.arm_indices = {}
        for name in arm_names[constraint_start_index:constraint_end_index]:
            nv.append(name)
            self.arm_indices[name] = len(self.arm_indices)

        ik_solver_updated = False
        self.ik_solvers = {}
        lb = []
        ub = []
        for c in constraints:
            for name, ee in zip(arm_names[constraint_start_index:constraint_end_index],ee_links[constraint_start_index:constraint_end_index]):
                ik_solver = c.add_trac_ik_adapter(name, base_link, ee, 0.1, 1e-6, desc)
                if not ik_solver_updated:
                    self.ik_solvers[name] = ik_solver
                    lb.append(ik_solver.get_lower_bound())
                    ub.append(ik_solver.get_upper_bound())

            ik_solver_updated = True

            c.set_max_iterations(2000)
            c.set_tolerance(5e-3)
            c.set_names(nv)


        # self.constraint.set_early_stopping(True)
        self.lb = np.concatenate(lb, axis=0)
        self.ub = np.concatenate(ub, axis=0)
        
        if planning_scene is None:
            self.planning_scene = PlanningScene(arm_names=arm_names, arm_dofs=arm_dofs, topic_name=planning_scene_name, **kwargs)

        self.T_og = None
        self.T_go = None
        # self.planning_scene = PlanningScene(names, [7]*len(names), **kwargs)

        # self.lb = self.ik_solvers.get_lower_bound()
        # self.ub = self.ik_solvers.get_upper_bound()
    
    def set_chains(self, chains):
        """setter for chains

        Args:
            chains (list of np.array): [7D (pos, quat) for each chain]
        """
        self.constraint.set_chains(np.concatenate(chains, axis=0))

        if self.constraint_ik is not None:
            self.constraint_ik.set_chains(np.concatenate(chains, axis=0))
        
    def set_chains_from_joints(self, q):
        """setter for chains

        Args:
            q (np.array): initial joint configurations that will be used to compute the chains
        """
        self.constraint.set_chains_from_joints(q)

        if self.constraint_ik is not None:
            self.constraint_ik.set_chains_from_joints(q)

    def forward_kinematics(self, name, q):
        q = q.astype(np.double)
        iso = self.ik_solvers[name].forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat
    
    def set_grasp_to_object_pose(self, go_pos=None, go_quat=None, T_go=None):
        if T_go is None:
            assert go_pos is not None and go_quat is not None, 'go_pos and go_quat must be provided'
            T_go = get_transform(go_pos, go_quat)
        else:
            self.T_go = T_go
        self.T_og = np.linalg.inv(T_go)

    def get_object_pose(self, q):
        """get object pose from joint configuration
        first arm is used to calculate the object pose

        Args:
            q (np.array): joint configuration

        Returns:
            np.array: object pose
        """
        pos, quat = self.forward_kinematics(self.arm_names[0], q[:7])
        T_0g = get_transform(pos, quat)
        T_0o = T_0g @ self.T_go
        
        pos, quat = get_pose(T_0o)
        return np.concatenate([pos, quat], axis=0)
        # return self.constraint.get_object_pose(q)

    ## old version (first gripper pose based)
    # def update_target(self, x):
    #     assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
    #     x = x.astype(np.double)
    #     pos, quat = x[:3], x[3:]
    #     self.constraint_ik.set_target_pose(pos, quat)

    # new version (object pose based)
    def update_target(self, x):
        if self.constraint_ik is None:
            print('[update_target] constraint_ik is not set')
            return
        
        assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
        x = x.astype(np.double)
        pos, quat = x[:3], x[3:]

        T_0o = get_transform(pos, quat)
        T_0g = T_0o @ self.T_og
        # print('T_0g', T_0g)
        # print('T_0o', T_0o)
        # print('self.T_og', self.T_og)
        pos, quat = get_pose(T_0g)
        self.constraint_ik.set_target_pose(pos, quat)


class MultiChainFixedOrientationConstraint(ConstraintBase, ConstraintIKBase):
    def __init__(self, arm_names, base_link, ee_links, arm_dofs=[], 
                 axis=0, reversed_axis=False, orientation_offset=np.identity(3), 
                 constraint_start_index=None, constraint_end_index=None, 
                 desc='/robot_description', planning_scene=None, 
                 planning_scene_name='/planning_scenes_suhan', 
                 **kwargs):
        
        self.arm_dofs = arm_dofs
        self.orientation_offset = orientation_offset
        self.reversed_axis = reversed_axis
        self.axis = axis
        self.orientation_vector = np.zeros(3)
        self.orientation_vector[axis] = 1
        
        if constraint_end_index is None:
            constraint_end_index = len(arm_names)
        if constraint_start_index is None:
            constraint_start_index = 0
        
        self.constraint_start_index = constraint_start_index
            
        len_arms = constraint_end_index-constraint_start_index

        super(MultiChainFixedOrientationConstraint, self).__init__('MultiChainFixedOrientationConstraint', dim_constraint=6*(len_arms-1)+2)
        self.dim_constraint = 6*(len_arms-1)+2
        self.dim_constraint_ik = 6*(len_arms)

        arm_dof = sum(arm_dofs[constraint_start_index:constraint_end_index])

        self.constraint = MultiChainWithFixedOrientationConstraint(arm_dof, self.dim_constraint)

        self.constraint.set_orientation_vector(self.orientation_vector)
        self.constraint.set_orientation_offset(orientation_offset)
        
        constraints = []
        constraints.append(self.constraint)

        if arm_dof > self.dim_constraint_ik:
            self.constraint_ik = MultiChainConstraintIK(arm_dof, self.dim_constraint_ik)
            self.constraint_ik.set_step_size(0.1)
            self.constraint_ik.set_early_stopping(True)
            constraints.append(self.constraint_ik)
        else:
            self.constraint_ik = None    

        self.arm_names = arm_names[constraint_start_index:constraint_end_index]
        nv = NameVector()
        self.arm_indices = {}
        for name in arm_names[constraint_start_index:constraint_end_index]:
            nv.append(name)
            self.arm_indices[name] = len(self.arm_indices)

        ik_solver_updated = False
        self.ik_solvers = {}
        lb = []
        ub = []

        for c in constraints:
            for name, ee in zip(arm_names[constraint_start_index:constraint_end_index],ee_links[constraint_start_index:constraint_end_index]):
                ik_solver = c.add_trac_ik_adapter(name, base_link, ee, 0.1, 1e-6, desc)
                if not ik_solver_updated:
                    self.ik_solvers[name] = ik_solver
                    lb.append(ik_solver.get_lower_bound())
                    ub.append(ik_solver.get_upper_bound())

            ik_solver_updated = True

            c.set_max_iterations(2000)
            c.set_tolerance(5e-4)
            c.set_names(nv)

        self.lb = np.concatenate(lb, axis=0)
        self.ub = np.concatenate(ub, axis=0)
        
        if planning_scene is None:
            self.planning_scene = PlanningScene(arm_names, arm_dofs=arm_dofs, topic_name=planning_scene_name, **kwargs)

        self.T_og = None
        self.T_go = None
    
    def set_chains(self, chains):
        """setter for chains

        Args:
            chains (list of np.array): [7D (pos, quat) for each chain]
        """
        self.constraint.set_chains(np.concatenate(chains, axis=0))

        if self.constraint_ik is not None:
            self.constraint_ik.set_chains(np.concatenate(chains, axis=0))
        
    def set_chains_from_joints(self, q):
        """setter for chains

        Args:
            q (np.array): initial joint configurations that will be used to compute the chains
        """
        self.constraint.set_chains_from_joints(q)

        if self.constraint_ik is not None:
            self.constraint_ik.set_chains_from_joints(q)

    def forward_kinematics(self, name, q):
        q = q.astype(np.double)
        iso = self.ik_solvers[name].forward_kinematics(q)
        d = isometry_to_vectors(iso)
        pos = d.first
        quat = d.second
        return pos, quat
    
    def set_grasp_to_object_pose(self, go_pos=None, go_quat=None, T_go=None):
        if T_go is None:
            assert go_pos is not None and go_quat is not None, 'go_pos and go_quat must be provided'
            T_go = get_transform(go_pos, go_quat)
            self.T_go = T_go
        else:
            self.T_go = T_go
        self.T_og = np.linalg.inv(T_go)

        self.constraint.set_orientation_offset(self.T_og[:3,:3])
        self.orientation_offset = self.T_og[:3,:3]



    def get_object_pose(self, q):
        """get object pose from joint configuration
        first arm is used to calculate the object pose

        Args:
            q (np.array): joint configuration

        Returns:
            np.array: object pose
        """
        pos, quat = self.forward_kinematics(self.arm_names[0], q[:7])
        T_0g = get_transform(pos, quat)
        T_0o = T_0g @ self.T_go
        
        pos, quat = get_pose(T_0o)
        return np.concatenate([pos, quat], axis=0)
        # return self.constraint.get_object_pose(q)

    ## old version (first gripper pose based)
    # def update_target(self, x):
    #     assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
    #     x = x.astype(np.double)
    #     pos, quat = x[:3], x[3:]
    #     self.constraint_ik.set_target_pose(pos, quat)

    # new version (object pose based)
    def update_target(self, x):
        if self.constraint_ik is None:
            print('[update_target] constraint_ik is not set')
            return
        assert len(x) == 7, 'x must be a 7d vector (pos 3, quat 4)'
        x = x.astype(np.double)
        pos, quat = x[:3], x[3:]

        T_0o = get_transform(pos, quat)
        T_0g = T_0o @ self.T_og

        pos, quat = get_pose(T_0g)
        self.constraint_ik.set_target_pose(pos, quat)

    def project(self, q):
        r = super().project(q)
        if r is False:
            return False
        
        r = self.check_orientation_side(q)
        return r
        
    def check_orientation_side(self, q):
        _, quat = self.forward_kinematics(self.arm_names[self.constraint_start_index], 
                                          q[:self.arm_dofs[self.constraint_start_index]])
        r = R.from_quat(quat)
        local_r = self.orientation_offset @ r.as_matrix().T

        if local_r[self.axis, self.axis] > 0.9:
            # True if not reversed
            return not self.reversed_axis
        
        elif local_r[self.axis, self.axis] < -0.9:
            # True if reversed
            return self.reversed_axis
        
        else:
            print('local_r', local_r)
            print('r', r.as_matrix())
            print('self.orientation_offset', self.orientation_offset)
            raise Exception("Something's wrong")
        
        return False
    
class ParallelChainConstraint(ConstraintBase):
    def __init__(self, links=3, chain_num=4, radius=1, length=1, joint_radius=0.2, **kwargs):
        """
        ParallelChainConstraint
        
        links: number of links in each kinematic chain. Minimum is 3. Must be odd.
        chain_num: number of chains in parallel mechanism. Minimum is 2.
        """
        # super(ParallelChainConstraint, self).__init__('ParallelChainConstraint', dim_constraint=dim_constraint)
        self.constraint = ParallelConstraint(links, chain_num, radius, length, joint_radius)
        self.constraint.set_early_stopping(True)
        self.q_dim = self.constraint.get_ambient_dimension()
        self.l_dim = self.constraint.get_co_dimension()
        self.dim_constraint = self.l_dim

        # print('q_dim', self.q_dim)
        # print('l_dim', self.l_dim)
        # self.constraint.set_max_iterations(2000)
        # self.constraint.set_tolerance(5e-4)

        self.lb = np.zeros(self.q_dim)
        self.ub = np.zeros(self.q_dim)

        for c in range(chain_num):
            o = 3 * c * links
            for i in range(links):
                self.lb[o + 3 * i + 0] = -i - 2
                self.lb[o + 3 * i + 1] = -i - 2
                self.lb[o + 3 * i + 2] = -i - 2

                self.ub[o + 3 * i + 0] = i + 2
                self.ub[o + 3 * i + 1] = i + 2
                self.ub[o + 3 * i + 2] = i + 2

    def get_start(self):
        q = np.zeros(self.q_dim)
        self.constraint.get_start(q)
        return q

    def get_goal(self):
        q = np.zeros(self.q_dim)
        self.constraint.get_goal(q)
        return q
        
    def is_valid(self, q):
        return self.constraint.is_valid(q)
