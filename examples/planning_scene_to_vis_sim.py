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

scene_bound_min = np.array([-1, -1, 0])
scene_bound_max = np.array([1, 1, 2])
vs.set_scene_bounds(scene_bound_min, scene_bound_max)
voxel_grid = vs.generate_voxel_occupancy()

voxel_grid = voxel_grid.reshape(16,16,16)
print(voxel_grid)

import matplotlib.pyplot as plt
title_font = {
    'fontsize': 16,
    'fontweight': 'bold'
}
ax1 = plt.figure(1).add_subplot()
ax1.set_title("depth image", fontsize=16, fontweight='bold', pad=20)
ax1.imshow(depth)

ax = plt.figure(2).add_subplot(projection='3d')
ax.voxels(voxel_grid)
ax.set_title("voxel grid", fontsize=16, fontweight='bold', pad=20)

plt.show()