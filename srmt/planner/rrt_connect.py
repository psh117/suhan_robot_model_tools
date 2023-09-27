import numpy as np

# from srmt.planning_scene import PlanningScene
from srmt.planner.abstact_planner import Planner
from srmt.planner.motion_tree import MotionTree

import random
from enum import Enum
import time
import copy

class GrowStatus(Enum):
    REACHED = 1
    TRAPPED = 2
    ADVANCED = 3

class SuhanRRTConnect(Planner):
    def __init__(self, state_dim=3, lb=None, ub=None, validity_fn=None, start_region_fn=None, goal_region_fn=None) -> None:
        
        # Please use validity_fn=planning_scene.is_valid if you want to use planning_scene
        # self.planning_scene = planning_scene

        self.validity_fn = validity_fn
        self.start_tree = MotionTree(name='start_tree', multiple_roots=True if start_region_fn is not None else False)
        self.goal_tree = MotionTree(name='goal_tree', multiple_roots=True if goal_region_fn is not None else False)
        self.start_q = None
        self.goal_q = None
        self.sampled_goals = []
        self.next_goal_index = 0
        self.distance_btw_trees = float('inf')
        self.state_dim = state_dim
        self.solved_time = -1.0
        
        self.p_sample = 0.05  # start or goal sampling probability

        self.start_region_fn = start_region_fn
        self.goal_region_fn = goal_region_fn
        
        if lb is None:
            lb = - np.ones(self.state_dim) * np.pi
        if ub is None:
            ub = np.ones(self.state_dim) * np.pi

        self.lb = lb
        self.ub = ub
        
        # properties
        self.max_distance = 0.1

        self.debug = True

    def print(self, *args):
        if self.debug:
            print(*args)
    
    def is_valid(self,q):
        if (q <self.lb).any() or (q > self.ub).any():
            return False
        # r = self.planning_scene.is_valid(q)
        r = self.validity_fn(q)
        
        return r

    def set_start(self, q):
        self.start_q = q

        if self.start_tree.multiple_roots:
            self.start_tree.add_root_node(q)
        else:
            self.start_tree.set_root(q)

    def set_goal(self, q):
        self.goal_q = q

        if self.goal_tree.multiple_roots:
            self.goal_tree.add_root_node(q)
        else:
            self.goal_tree.set_root(q)

    def solve(self, max_time=10.0):
        start_time = time.time()
        solved = False
        is_start_tree = True
        self.distance_btw_trees = float('inf')
        terminate = False
        
        if self.start_q is None:
            if self.start_tree.multiple_roots:
                start_q = self.start_region_fn()
                self.set_start(start_q)
            else:
                raise ValueError('start_q is None, but start_tree.multiple_roots is False') 

        if self.goal_q is None:
            if self.goal_tree.multiple_roots:
                goal_q = self.goal_region_fn()
                self.set_goal(goal_q)
            else:
                raise ValueError('goal_q is None, but goal_tree.multiple_roots is False')

        while terminate is False: # TODO: time condition ?
            if (time.time() - start_time) > max_time:
                self.print('timed out')
                break

            if self.start_region_fn is not None:
                if self.start_tree.multiple_roots:
                    if random.random() < self.p_sample:
                        start_q = self.start_region_fn()
                        self.start_tree.add_root_node(start_q)

                        self.print('added start', start_q)
            
            if self.goal_region_fn is not None:
                if self.goal_tree.multiple_roots:
                    if random.random() < self.p_sample:
                        goal_q = self.goal_region_fn()
                        self.goal_tree.add_root_node(goal_q)

                        self.print('added goal', goal_q)

            if is_start_tree:
                cur_tree = self.start_tree
                other_tree = self.goal_tree
            else:
                cur_tree = self.goal_tree
                other_tree = self.start_tree
            
            # sample
            q_rand = self.random_sample()
            r, q_des, q_idx_cur = self.grow(cur_tree, q_rand)

            if r != GrowStatus.TRAPPED:
                q_added = copy.deepcopy(q_des)

                if r != GrowStatus.REACHED:
                    q_rand = copy.deepcopy(q_des)

                is_local_start = is_start_tree
                r, q_des, q_idx = self.grow(other_tree, q_rand)
                
                # if r == GrowStatus.TRAPPED:
                #     pass
                while r == GrowStatus.ADVANCED:
                    r, q_des, q_idx = self.grow(other_tree, q_rand)
                    
                q_near_other, q_idx_other = other_tree.get_nearest(q_rand)
                new_dist = self.distance(q_rand, q_near_other)
                if new_dist < self.distance_btw_trees:
                    self.distance_btw_trees = new_dist
                    self.print ('Estimated distance to go: {0}'.format(new_dist))

                if r == GrowStatus.REACHED:
                    
                    end_time = time.time()
                    elapsed = end_time - start_time
                    self.print ('found a solution! elapsed_time:',elapsed)
                    self.solved_time = elapsed

                    cur_path = cur_tree.get_path(q_idx_cur)
                    other_path = other_tree.get_path(q_idx_other)

                    if is_start_tree:
                        q_path = cur_path + list(reversed(other_path))
                    else:
                        q_path = other_path + list(reversed(cur_path))

                    q_path = np.array(q_path)

                    return True, q_path
        
            is_start_tree = not is_start_tree

        return False, None

    def distance(self, q1, q2):
        dist = np.linalg.norm(q1 - q2)
        return dist

    def random_sample(self):
        q = np.random.uniform(self.lb, self.ub)
        return q

    def grow(self, tree, q):
        q_near, q_near_idx = tree.get_nearest(q)

        dist = self.distance(q_near, q)
        if dist > self.max_distance:
            q_int = self.interpolate(q_near, q, self.max_distance / dist) # TODO: check ratio

            q_des = q_int
            reach = False
        else:
            q_des = q
            reach = True


        if self.is_valid(q_des) is False:
            return GrowStatus.TRAPPED, q_des, 0
        
        q_idx = tree.add_node(q_near_idx, q_des)
        
        if reach:
            return GrowStatus.REACHED, q_des, q_idx
        
        return GrowStatus.ADVANCED, q_des, q_idx

    def interpolate(self, q_from, q_to, ratio):
        q_int = (q_to-q_from) * ratio + q_from
        
        return q_int
