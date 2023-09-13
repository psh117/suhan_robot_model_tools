from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import TRACIKAdapter, vectors_to_isometry, isometry_to_vectors
import numpy as np

class TRACIK(object):
    def __init__(self, base_link, tip_link, max_time=0.1, precision=1e-5, urdf_param='/robot_description'):
        self.tracik = TRACIKAdapter(base_link, tip_link, max_time, precision, urdf_param)

    def solve(self, pos, quat, q_init):
        assert(self.tracik.get_num_joints() == len(q_init), 'q_init size mismatch')

        iso = vectors_to_isometry(pos, quat)
        q_res = np.zeros(self.tracik.get_num_joints())
        r = self.tracik.solve(q_init, iso, q_res)
        return r, q_res

    def forward_kinematics(self, q):
        pos_quat = isometry_to_vectors(self.tracik.forward_kinematics(q))
        pos = pos_quat.first
        quat = pos_quat.second
        return pos, quat

    def get_lower_bound(self):
        return self.tracik.get_lower_bound()

    def get_upper_bound(self):  
        return self.tracik.get_upper_bound()

    def get_num_joints(self):
        return self.tracik.get_num_joints()

    def is_valid(self, q):
        return self.tracik.is_valid(q)

    def get_jacobian_matrix(self, q):
        return self.tracik.get_jacobian_matrix(q)

    def set_bounds(self, lb, ub):
        self.tracik.set_bounds(lb, ub)

    def set_tolerance_bounds(self, tol):
        self.tracik.set_tolerance_bounds(tol)

