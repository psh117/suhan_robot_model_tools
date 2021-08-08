import os
import time
import torch
import pickle
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict
from torch._C import device

from torchvision import transforms
import numpy as np
import scipy.io
import math
from math import pi

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torch.utils.data
from torch.utils.data import Dataset
from torch.utils.data import DataLoader

from hyperspherical_vae.distributions import VonMisesFisher
from hyperspherical_vae.distributions import HypersphericalUniform

from suhan_robot_model_tools import suhan_robot_model_tools_wrapper_cpp
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

from sensor_msgs.msg import JointState

import rospy

q_dim = 21
constraint_dim = 12
latent_dim = q_dim - constraint_dim + 1

def denormalize(norm):
    joint_limits_ub = np.array([pi] * 21)
    joint_limits_lb = np.array([-pi] * 21)
    epsilon = 0.00
    joint_len = joint_limits_ub - joint_limits_lb - (epsilon * 2)

    q = norm * joint_len + joint_limits_lb

    return q

def normalize(q):
    joint_limits_ub = np.array([pi] * 21)
    joint_limits_lb = np.array([-pi] * 21)
    epsilon = 0.00
    joint_len = joint_limits_ub - joint_limits_lb - (epsilon * 2)

    norm = (q - joint_limits_lb) / joint_len
    return norm

class ModelVAE(torch.nn.Module):
    
    def __init__(self, h_dim, z_dim, activation=F.elu, distribution='normal', device='cpu'):
        """
        ModelVAE initializer
        :param h_dim: dimension of the hidden layers
        :param z_dim: dimension of the latent representation
        :param activation: callable activation function
        :param distribution: string either `normal` or `vmf`, indicates which distribution to use
        """
        super(ModelVAE, self).__init__()
        
        self.z_dim, self.activation, self.distribution = z_dim, activation, distribution
        
        # 2 hidden layers encoder
        self.fc_e0 = nn.Linear(q_dim, h_dim * 2)#.to(device)
        self.fc_e1 = nn.Linear(h_dim * 2, h_dim * 2)#.to(device)
        self.fc_e2 = nn.Linear(h_dim * 2, h_dim)#.to(device)

        if self.distribution == 'normal':
            # compute mean and std of the normal distribution
            self.fc_mean = nn.Linear(h_dim, z_dim)#.to(device)
            self.fc_var =  nn.Linear(h_dim, z_dim)#.to(device)
        elif self.distribution == 'vmf':
            # compute mean and concentration of the von Mises-Fisher
            self.fc_mean = nn.Linear(h_dim, z_dim)#.to(device)
            self.fc_var = nn.Linear(h_dim, 1)#.to(device)
        else:
            raise NotImplemented
            
        # 2 hidden layers decoder
        self.fc_d0 = nn.Linear(z_dim, h_dim)#.to(device)
        self.fc_d1 = nn.Linear(h_dim, h_dim * 2)#.to(device)
        self.fc_d2 = nn.Linear(h_dim* 2, h_dim * 2)#.to(device)
        self.fc_logits = nn.Linear(h_dim * 2, q_dim)#.to(device)
        self.device = device

    def encode(self, x):
        # 2 hidden layers encoder
        device = self.device
        x = self.activation(self.fc_e0(x))#.to(device)
        x = self.activation(self.fc_e1(x))#.to(device)
        x = self.activation(self.fc_e2(x))#.to(device)
        
        if self.distribution == 'normal':
            # compute mean and std of the normal distribution
            z_mean = self.fc_mean(x)
            z_var = F.softplus(self.fc_var(x))#.to(device)
        elif self.distribution == 'vmf':
            # compute mean and concentration of the von Mises-Fisher
            z_mean = self.fc_mean(x)
            # print(z_mean.norm(dim=-1, keepdim=True))
            z_mean = z_mean / z_mean.norm(dim=-1, keepdim=True)
            # the `+ 1` prevent collapsing behaviors
            z_var = F.softplus(self.fc_var(x)) + 1
        else:
            raise NotImplemented
        
        return z_mean, z_var
        
    def decode(self, z):
        
        device = self.device
        x = self.activation(self.fc_d0(z))
        x = self.activation(self.fc_d1(x))
        x = self.activation(self.fc_d2(x))
        x = F.sigmoid(self.fc_logits(x))
        
        return x
        
    def reparameterize(self, z_mean, z_var):
        device = self.device
        if self.distribution == 'normal':
            q_z = torch.distributions.normal.Normal(z_mean, z_var)#.to(device)
            p_z = torch.distributions.normal.Normal(torch.zeros_like(z_mean), torch.ones_like(z_var))#.to(device)
        elif self.distribution == 'vmf':
            q_z = VonMisesFisher(z_mean, z_var)
            p_z = HypersphericalUniform(self.z_dim - 1, device=device)#.to(device)
            # p_z.device = device
        else:
            raise NotImplemented

        return q_z, p_z
        
    def forward(self, x): 
        device = self.device
        z_mean, z_var = self.encode(x)
        q_z, p_z = self.reparameterize(z_mean, z_var)
        z = q_z.rsample()#.to(device)
        x_ = self.decode(z)
        
        return (z_mean, z_var), (q_z, p_z), z, x_
    
    def to_latent(self,q):
        device = 'cpu'
        q = normalize(q)
        # print('raw',q)
        q = torch.from_numpy(q[None,:]).float().to(device)
        z, _ = self.encode(x=q)
        z = z.cpu().detach().numpy()[0]
        return z

    def from_latent(self,z):
        device = 'cpu'
        z = torch.from_numpy(z[None,:]).float().to(device)
        val = self.decode(z=z).cpu().detach().numpy()[0]
        # print('raw',val)
        q = denormalize(val)
        return q
        
joint_limits_ub = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973] * 3)
joint_limits_lb = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973] * 3)

def main(args):
    model_name = 'checkpoint_epoch_1423_assembly_joint_manifold_vmf.pkl'
    device = 'cpu'
    model = ModelVAE(h_dim=512, z_dim=latent_dim, distribution='vmf',device=device).to(device)
    # z_rand = np.random.rand(latent_dim)
    # z_rand = z_rand / np.linalg.norm(z_rand)

    # print('z_rand', z_rand)
    model.load_state_dict(torch.load(model_name, map_location=torch.device('cpu')))
    model.eval()

    robots = ['panda_left', 'panda_right', 'panda_top']
    joint_names = []
    for robot in robots:
        for i in range(1,8):
            name = '{0}_joint{1}'.format(robot, i)
            joint_names.append(name)
    print(joint_names)
    msg = JointState()
    msg.name = joint_names
    msg.velocity = [] * 21
    msg.effort = [] * 21

    rospy.init_node('suhan2',anonymous=True)
    roscpp_init('suhan', [])
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    tcc = suhan_robot_model_tools_wrapper_cpp.TripleChainConstraintsFunctions()
    tcc.add_trac_ik_adapter('left_arm', 'base','panda_left_hand', 0.1, 1e-6,  '/robot_description')
    tcc.add_trac_ik_adapter('right_arm', 'base','panda_right_hand', 0.1, 1e-6,  '/robot_description')
    tcc.add_trac_ik_adapter('top_arm', 'base','panda_top_hand', 0.1, 1e-6,  '/robot_description')
    tcc.set_names('left_arm', 'right_arm', 'top_arm')

    q_init = np.array([ -0.974319515483986,0.502209761399138,0.405927984735235,-1.62410050562666, 0.00176466876963385, 2.31114958061098,-0.372115841715219,
            -0.868238261722017,0.553539761670335, 1.01975929351465,-1.78394452707266,-0.611703083258935,  2.32864727930383, 0.666091265175776, 
            0.700846673688826, 0.586681050244243,-0.367622704567015, -1.21504380136177,  0.25971726786195,  1.49314527452455, 0.664837089101801])

    tcc.set_chain(q_init[:7], q_init[7:14], q_init[14:])
    tcc.set_max_iterations(4000)
    tcc.set_tolerance(1e-4)
    tcc.set_num_finite_diff(3) #3, 5, 7
    val = np.array([0.0])

    rate = rospy.Rate(100)
    while rospy.is_shutdown() is False:
        z_rand = np.random.rand(latent_dim) * 2.0 - 1.0
        z_rand = z_rand / np.linalg.norm(z_rand)
        q = model.from_latent(z_rand)
        if (q > joint_limits_ub).any():
            continue
        if (q < joint_limits_lb).any():
            continue
        tcc.function(q,val)
        if val[0] >1.5:
            continue
        break
    # display z_rand move (jl)
    while rospy.is_shutdown() is False:
        z_new = np.random.rand(latent_dim) * 2.0 - 1.0
        z_new = z_new / np.linalg.norm(z_new)
        q = model.from_latent(z_new)
        if (q > joint_limits_ub).any():
            continue
        if (q < joint_limits_lb).any():
            continue
        for _ in range(100):
            z_diff = (z_rand-z_new)
            z_rand += z_diff / np.linalg.norm(z_diff) * 0.01
            # z_rand += (np.random.rand(latent_dim) * 2.0 - 1.0) * 0.001
            z_rand = z_rand / np.linalg.norm(z_rand)

            # print(z_rand)
            q = model.from_latent(z_rand)
            # tcc.project(q)
            # if (q > joint_limits_ub).any():
            #     continue
            # if (q < joint_limits_lb).any():
            #     continue
            
            tcc.function(q,val)
            if val[0] >4.5:
                continue
            print(val, q)
            tcc.project(q)

            msg.position = q.tolist()
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            rate.sleep()
    # # display z_rand move 
    # while rospy.is_shutdown() is False:
    #     z_new = np.random.rand(latent_dim) * 2.0 - 1.0
    #     for _ in range(100):
    #         z_diff = (z_rand-z_new)
    #         z_rand += z_diff / np.linalg.norm(z_diff) * 0.01
    #         # z_rand += (np.random.rand(latent_dim) * 2.0 - 1.0) * 0.001
    #         z_rand = z_rand / np.linalg.norm(z_rand)

    #         print(z_rand)
    #         q = model.from_latent(z_rand)
    #         tcc.project(q)
    #         # if (q > joint_limits_ub).any():
    #         #     continue
    #         # if (q < joint_limits_lb).any():
    #         #     continue
            
    #         tcc.function(q,val)
    #         # print(val, q)
    #         msg.position = q.tolist()
    #         msg.header.stamp = rospy.Time.now()
    #         pub.publish(msg)
    #         rate.sleep()
    # # display z_rand move 
    # while rospy.is_shutdown() is False:
    #     z_new = np.random.rand(latent_dim) * 2.0 - 1.0
    #     for _ in range(100):
    #         z_diff = (z_rand-z_new)
    #         z_rand += z_diff / np.linalg.norm(z_diff) * 0.001
    #         # z_rand += (np.random.rand(latent_dim) * 2.0 - 1.0) * 0.001
    #         z_rand = z_rand / np.linalg.norm(z_rand)

    #         print(z_rand)
    #         q = model.from_latent(z_rand)
    #         # if (q > joint_limits_ub).any():
    #         #     continue
    #         # if (q < joint_limits_lb).any():
    #         #     continue
            
    #         tcc.function(q,val)
    #         # print(val, q)
    #         msg.position = q.tolist()
    #         msg.header.stamp = rospy.Time.now()
    #         pub.publish(msg)
    #         rate.sleep()

    # # projection assured 
    # while rospy.is_shutdown() is False:
    #     z_rand = np.random.rand(latent_dim) * 2.0 - 1.0
    #     z_rand = z_rand / np.linalg.norm(z_rand)

    #     # print('z_rand', z_rand)

    #     q = model.from_latent(z_rand)
    #     if (q > joint_limits_ub).any():
    #         continue
    #     if (q < joint_limits_lb).any():
    #         continue
    #     r = tcc.project(q)
    #     if r is False:
    #         continue
    #     if (q > joint_limits_ub).any():
    #         continue
    #     if (q < joint_limits_lb).any():
    #         continue
        
    #     tcc.function(q,val)
    #     print(val, q)
    #     msg.position = q.tolist()
    #     msg.header.stamp = rospy.Time.now()
    #     pub.publish(msg)
    #     input()
        
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--epochs", type=int, default=10000)
    parser.add_argument("--batch_size", type=int, default=256)
    parser.add_argument("--learning_rate", type=float, default=0.00005)
    parser.add_argument("--latent_size", type=int, default=5)
    parser.add_argument("--print_every", type=int, default=100)
    parser.add_argument("--fig_root", type=str, default='figs')
    parser.add_argument("--conditional", type=bool, default='True')
    parser.add_argument("--object", type=str, default='ikea_stefan_middle')

    args = parser.parse_args()

    main(args)