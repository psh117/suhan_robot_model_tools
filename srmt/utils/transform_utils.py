
import numpy as np
import scipy

def get_transform(p, q):
    rot = scipy.spatial.transform.Rotation.from_quat(q).as_matrix()

    T = np.eye(4)
    T[:3,:3] = rot
    T[:3,3] = p

    return T
    
def get_pose(T):
    p = T[:3,3]
    q = scipy.spatial.transform.Rotation.from_matrix(T[:3,:3]).as_quat()

    return p, q