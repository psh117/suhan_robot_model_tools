from srmt.planning_scene import PlanningScene, VisualSimulator

import numpy as np

pc = PlanningScene(arm_names=['panda_left', 'panda_right', 'panda_top'], dofs=[7,7,7])
pc.add_box('abcd', [.1,0.1,0.5], [-0.3,-0.1,1.1], [0.0,0.0,0.0,1.0])
pc.add_box('abcd2', [0.2,0.5,0.1], [0.2,0.1,1.1], [0.0,0.0,0.0,1.0])

vs = VisualSimulator()
vs.load_scene(pc)

vs.set_cam_and_target_pose(np.array([0.5, 0.0, 2.0]), np.array([0.0, 0.0, 1.0]))
np.printoptions(precision=3, suppress=True, linewidth=100, threshold=10000)
depth = vs.generate_depth_image()

print(depth)

import matplotlib.pyplot as plt
plt.imshow(depth)
plt.show()
# print(max(depth.flatten()))
# print(min(depth.flatten()))