from suhan_robot_model_tools.suhan_robot_model_tools_wrapper_cpp import VisualSim, isometry_to_vectors, vectors_to_isometry
from .planning_scene import PlanningScene
import rospy
import copy
import numpy as np


class VisualSimulator(object):
    def __init__(self) -> None:
        # super().__init__()
        self.vs = VisualSim()
        pass

    def load_scene(self, planning_scene : PlanningScene):
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