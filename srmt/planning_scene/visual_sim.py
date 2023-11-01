from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import VisualSim, isometry_to_vectors, vectors_to_isometry
from .planning_scene import PlanningSceneLight
import rospy
import copy
import numpy as np


class VisualSimulator(object):
    def __init__(self, n_grid=32) -> None:
        # super().__init__()
        self.vs = VisualSim()
        self.vs.set_grid_resolution(n_grid)
        pass

    def load_scene(self, planning_scene : PlanningSceneLight):
        self.vs.load_scene(planning_scene.pc.get_planning_scene())

    def set_cam_pose(self, pos, quat):
        iso = vectors_to_isometry(pos, quat)
        self.vs.set_cam_pose(iso)

    def set_cam_pos(self, pos):
        if pos is list:
            pos = np.array(pos)
        self.vs.set_cam_pos(pos)
    
    def lookat(self, target_pos):
        if target_pos is list:
            target_pos = np.array(target_pos)
        self.vs.lookat(target_pos)

    def set_cam_and_target_pose(self, cam_pos, target_pos):
        self.set_cam_pos(cam_pos)
        self.lookat(target_pos)

    def generate_depth_image(self):
        return self.vs.generate_depth_image()
    
    def generate_voxel_occupancy(self):
        return self.vs.generate_voxel_occupancy()

    def generate_point_cloud_matrix(self):
        """
        Returns:
            np.ndarray: (n_points, 3) matrix
        """
        return self.vs.generate_point_cloud_matrix()
    
    def generate_local_voxel_occupancy(self, point_cloud_matrix, 
                                       obj_pos, obj_quat, 
                                       obj_bound_min, obj_bound_max, 
                                       n_grids=np.array([32,32,32]), 
                                       near_distance=0.2, 
                                       fill_occluded_voxels=False):
        """
        Args:
            point_cloud_matrix (np.ndarray): (n_points, 3) matrix
            obj_pos (np.ndarray): (3,) vector
            obj_quat (np.ndarray): (4,) vector
            obj_bound_min (np.ndarray): (3,) vector
            obj_bound_max (np.ndarray): (3,) vector
            n_grids (np.ndarray): (3,) vector of the number of grids in each dimension of the local voxel occupancy
            near_distance (float): distance to the object that will be used to generate the local voxel occupancy
            fill_occluded_voxels (bool): whether to fill the occluded voxels to be occupied
        Returns:
            np.ndarray: (n_grids[0], n_grids[1], n_grids[2]) matrix
        """
        obj_pose = vectors_to_isometry(obj_pos, obj_quat)
        return_vec = self.vs.generate_local_voxel_occupancy(point_cloud_matrix, obj_pose, obj_bound_min, obj_bound_max, n_grids, near_distance, fill_occluded_voxels)

        return return_vec.reshape(int(n_grids[0]), int(n_grids[1]), int(n_grids[2]))

    def set_scene_bounds(self, scene_bound_min, scene_bound_max):
        return self.vs.set_scene_bounds(scene_bound_min, scene_bound_max)

    def set_grid_resolution(self, n_grid):
        return self.vs.set_grid_resolution(n_grid)