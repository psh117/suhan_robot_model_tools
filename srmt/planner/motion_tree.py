
from anytree import Node

import numpy as np


class MotionTree():
    def __init__(self, name = '', multiple_roots = False) -> None:
        self.nodes = [None]
        self.name = name
        self.multiple_roots = multiple_roots
        self.grow_fail_count = 0
        if multiple_roots:
            self.nodes = [Node(name='root')]

    def __len__(self):
        if self.multiple_roots:
            return len(self.nodes) - 1
        
        return len(self.nodes)

    def set_root(self, init_x):
        self.nodes = [Node(name='root', x=init_x)]

    def add_root_node(self, x):
        return self.add_node(0, x)

    def get_root(self):
        return self.nodes[0]

    def add_node(self, parent_index, x):
        self.nodes.append(Node(name='n_{0}'.format(len(self.nodes)), x=x, parent=self.nodes[parent_index]))
        return len(self.nodes) - 1 # return index

    def get_nearest(self, x):
        """
        return: q, tree node index
        """
        if self.multiple_roots:
            dists = np.array([(np.linalg.norm(x-node.x)) for node in self.nodes[1:]])    
            min_index = dists.argmin() + 1
        else: 
            dists = np.array([(np.linalg.norm(x-node.x)) for node in self.nodes])
            min_index = dists.argmin()

        # print (dists)

        return self.nodes[min_index].x, min_index

    def get_path(self, index):

        if self.multiple_roots:
            x_path = [node.x for node in self.nodes[index].path[1:]]
        else:
            x_path = [node.x for node in self.nodes[index].path]

        return x_path

