from srmt.planning_scene import PlanningScene

import numpy as np
import random
import math

def add_table(pc, pos, dphi, dtheta, length, width, height, d):
    quat = [0.0, 0.0, 0.0, 1.0]
    posf = translation(pos, dphi) 
    quatf = rotation(quat, dtheta)

    table_top_dim = [length, width, d] #[0.5,0.5,0.05]
    table_leg_dim = [d, d, height] #[0.1,0.1, 0.2]

    ### dphi -> rot
    table_top_pose = transform([posf[0], posf[1], posf[2]+height/2-d/2], dtheta, posf)
    table_leg_left_back_pose0 = [posf[0]-length/2+d/2, posf[1]+width/2-d/2, posf[2]]
    table_leg_left_front_pose0 = [posf[0]-length/2+d/2, posf[1]-width/2+d/2, posf[2]]
    table_leg_right_back_pose0 = [posf[0]+length/2-d/2, posf[1]+width/2-d/2, posf[2]]
    table_leg_right_front_pose0 = [posf[0]+length/2-d/2, posf[1]-width/2+d/2, posf[2]]
    table_leg_left_back_pose = transform(table_leg_left_back_pose0, dtheta, posf)
    table_leg_left_front_pose = transform(table_leg_left_front_pose0, dtheta, posf)
    table_leg_right_back_pose = transform(table_leg_right_back_pose0, dtheta, posf)
    table_leg_right_front_pose = transform(table_leg_right_front_pose0, dtheta, posf)
    # print('orig_table_leg_left_back_pose', [posf[0]-length/2+d/2, posf[1]+width/2-d/2, posf[2]])
    # print('table_leg_left_back_pose', table_leg_left_back_pose)
    pc.add_box('table_top', table_top_dim, table_top_pose, quatf)
    pc.add_box('table_leg_left_back', table_leg_dim, table_leg_left_back_pose, quatf)
    pc.add_box('table_leg_left_front', table_leg_dim, table_leg_left_front_pose, quatf)
    pc.add_box('table_leg_right_back', table_leg_dim, table_leg_right_back_pose, quatf)
    pc.add_box('table_leg_right_front', table_leg_dim, table_leg_right_front_pose, quatf)

    # for i in range(num_box):
    #     box_dim = [random.uniform(0.1, 0.3), random.uniform(0.1, 0.3), random.uniform(0.1, 0.3)]
    #     box_pos_xmin = min(table_leg_left_back_pose0[0], table_leg_left_front_pose0[0], table_leg_right_back_pose0[0], table_leg_right_front_pose0[0])+box_dim[0]/2
    #     box_pos_ymin = min(table_leg_left_back_pose0[1], table_leg_left_front_pose0[1], table_leg_right_back_pose0[1], table_leg_right_front_pose0[1])+box_dim[1]/2
    #     box_pos_xmax = max(table_leg_left_back_pose0[0], table_leg_left_front_pose0[0], table_leg_right_back_pose0[0], table_leg_right_front_pose0[0])-box_dim[0]/2
    #     box_pos_ymax = max(table_leg_left_back_pose0[1], table_leg_left_front_pose0[1], table_leg_right_back_pose0[1], table_leg_right_front_pose0[1])-box_dim[1]/2
    #     box_pos =  transform([random.uniform(box_pos_xmin, box_pos_xmax), random.uniform(box_pos_ymin, box_pos_ymax), table_top_pose[2]+box_dim[2]/2], dtheta, posf)
    #     pc.add_box('box'+str(i), box_dim, box_pos, quatf)


def add_shelf(pc, pos, dphi, dtheta, length, width, height, d, shelf_parts, id):
    quat = [0.0, 0.0, 0.0, 1.0]
    posf = translation(pos, dphi) 
    quatf = rotation(quat, dtheta)

    top_dim = [length, width, d] # mid_dim, bottom_dim
    left_dim = [d, width, height] # = right_dim
    back_dim = [length, d, height]
    top_pos =   transform([posf[0], posf[1], posf[2]+height/2-d/2], dtheta, posf)
    bottom_pos= transform([posf[0], posf[1], posf[2]-height/2+d/2], dtheta, posf)
    left_pos = transform([posf[0]-length/2+d/2, posf[1], posf[2]], dtheta, posf)
    right_pos = transform([posf[0]+length/2-d/2, posf[1], posf[2]], dtheta, posf)
    back_pos = transform([posf[0], posf[1]+width/2-d/2, posf[2]], dtheta, posf)

    pc.add_box('top'+str(id), top_dim, top_pos, quatf)
    pc.add_box('bottom'+str(id), top_dim, bottom_pos, quatf)
    for i in range(shelf_parts-1):
        mid_pos = transform([posf[0], posf[1], posf[2]-height/2+d/2+((height-d)/shelf_parts)*(i+1)], dtheta, posf)
        pc.add_box('mid'+str(id)+str(i+1), top_dim, mid_pos, quatf)
    pc.add_box('left'+str(id), left_dim, left_pos, quatf)
    pc.add_box('right'+str(id), left_dim, right_pos, quatf)
    pc.add_box('back'+str(id), back_dim, back_pos, quatf)
    
    # cup_pose = transform([posf[0], posf[1], posf[2]-height/2+d/2+((height-d)/shelf_parts)*(cup_pos-1)+d/2+cup_height/2], dtheta, posf)
    # pc.add_cylinder('cup'+str(id), cup_height, cup_radius, cup_pose, [0.0,0.0,0.0,1.0]) # cup size 

def transform(pos, theta, center):
    x0, y0, z0 = pos[0], pos[1], pos[2]
    xc, yc, zc = center[0], center[1], center[2]
    # rc = np.sqrt(xc**2 + yc**2)
    # thetac = np.arctan2(yc, xc)
    r0c = np.sqrt((x0-xc)**2 + (y0-yc)**2)
    theta0c = np.arctan2((y0-yc), (x0-xc))

    posf = [xc+r0c*np.cos(theta0c+theta), yc+r0c*np.sin(theta0c+theta), z0]
    # posf = [r*np.cos(phi0+dphi)+r1*np.cos(phi1), r*np.sin(phi0+dphi)+r1*np.sin(phi1), z0]
    # print('posf : ', posf)
    return posf

def translation(pos, dphi):
    x0, y0, z0 = pos[0], pos[1], pos[2]
    r = np.sqrt(x0**2 + y0**2)
    phi0 = np.arctan2(y0, x0)
    posf = [r*np.cos(phi0+dphi), r*np.sin(phi0+dphi), z0]
    return posf

def rotation(quat, dyaw = 0):
    droll, dpitch = 0, 0
    qx0 = quat[0]
    qy0 = quat[1]
    qz0 = quat[2]
    qw0 = quat[3]
    sinr_cosp = 2 * (qw0 * qx0 + qy0 * qz0)
    cosr_cosp = 1 - 2 * (qx0 * qx0 + qy0 * qy0)
    roll0 = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (qw0 * qy0 - qx0 * qz0))
    cosp = math.sqrt(1 - 2 * (qw0 * qy0 - qx0 * qz0))
    pitch0 = 2 * math.atan2(sinp, cosp) - np.pi / 2

    siny_cosp = 2 * (qw0 * qz0 + qx0 * qy0)
    cosy_cosp = 1 - 2 * (qy0 * qy0 + qz0 * qz0)
    yaw0 = math.atan2(siny_cosp, cosy_cosp)

    cr = np.cos(roll0+droll * 0.5)
    sr = np.sin(roll0+droll * 0.5)
    cp = np.cos(pitch0+dpitch * 0.5)
    sp = np.sin(pitch0+dpitch * 0.5)
    cy = np.cos(yaw0+dyaw * 0.5)
    sy = np.sin(yaw0+dyaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    q = [qx, qy, qz, qw]
    return q