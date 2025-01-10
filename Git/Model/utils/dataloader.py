"""
Module Name: .mat file Data Loader

Purpose:
    This module loads training data and validation data from .mat file from given base directory
    Its methods include returning the length of data and get the data in given index
"""

import torch
from torch.utils.data import DataLoader, Dataset
import numpy as np
import os
import h5py
import scipy.io

class MatDataLoader(Dataset):
    def __init__(self, data_length, base_dir):
        super(MatDataLoader, self).__init__()
        
        self.data_length = data_length
        self.base_dir = base_dir
        
        #self.t = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_x.mat"), "r")["data_x"][3:data_length, 0]).unsqueeze(-1).float()                                            # shape: batch_size x 1                  
        
        self.theta_1 = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle1.mat"), "r")["angle1"][3:data_length, 1])).unsqueeze(-1)                            
        self.theta_2 = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle2.mat"), "r")["angle2"][3:data_length, 1])).unsqueeze(-1)
        self.theta_3 = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle3.mat"), "r")["angle3"][3:data_length, 1])).unsqueeze(-1)
        self.theta =  torch.cat([self.theta_1, self.theta_2, self.theta_3], dim = 1)                                                                                                      # shape: batch_size x 3
                        
        self.theta_1_dot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle1_dot.mat"), "r")["angle1_dot"][3:data_length, 1])).unsqueeze(1)
        self.theta_2_dot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle2_dot.mat"), "r")["angle2_dot"][3:data_length, 1])).unsqueeze(1)
        self.theta_3_dot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle3_dot.mat"), "r")["angle3_dot"][3:data_length, 1])).unsqueeze(1)
        self.theta_dot =  torch.cat([self.theta_1_dot, self.theta_2_dot, self.theta_3_dot], dim = 1)                                                                                          # shape: batch_size x 3
        
        self.theta_1_Ddot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle1_Ddot.mat"), "r")["angle1_Ddot"][3:data_length, 1])).unsqueeze(1)
        self.theta_2_Ddot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle2_Ddot.mat"), "r")["angle2_Ddot"][3:data_length, 1])).unsqueeze(1)
        self.theta_3_Ddot = torch.deg2rad(torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_angle3_Ddot.mat"), "r")["angle3_Ddot"][3:data_length, 1])).unsqueeze(1)
        self.theta_Ddot =  torch.cat([self.theta_1_Ddot, self.theta_2_Ddot, self.theta_3_Ddot], dim = 1)                                                                                       # shape: batch_size x 3
        
        self.x = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_x.mat"), "r")["data_x"][3:data_length, 1]).unsqueeze(1)               
        self.y = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_y.mat"), "r")["data_y"][3:data_length, 1]).unsqueeze(1)  
        self.z = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_z.mat"), "r")["data_z"][3:data_length, 1]).unsqueeze(1)  
        self.s =  torch.cat([self.x * 0.001, self.y * 0.001, self.z * 0.001], dim = 1)                                                                                                                                # shape: batch_size x 3
        
        self.x_Ddot = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_x_Ddot.mat"), "r")["data_x_Ddot"][3:data_length, 1]).unsqueeze(1)  
        self.y_Ddot = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_y_Ddot.mat"), "r")["data_y_Ddot"][3:data_length, 1]).unsqueeze(1)  
        self.z_Ddot = torch.tensor(h5py.File(os.path.join(self.base_dir, "simulated_z_Ddot.mat"), "r")["data_z_Ddot"][3:data_length, 1]).unsqueeze(1)  
        self.s_Ddot =  torch.cat([self.x_Ddot * 0.001, self.y_Ddot * 0.001, self.z_Ddot * 0.001], dim = 1)                                                                                                             # shape: batch_size x 3
        
        self.tau_1 = torch.tensor(scipy.io.loadmat(os.path.join(self.base_dir, "tau_2.mat"))["tau_first_row"][0, 3:data_length]).unsqueeze(1)
        self.tau_2 = torch.tensor(scipy.io.loadmat(os.path.join(self.base_dir, "tau_2.mat"))["tau_second_row"][0, 3:data_length]).unsqueeze(1)
        self.tau_3 = torch.tensor(scipy.io.loadmat(os.path.join(self.base_dir, "tau_2.mat"))["tau_third_row"][0, 3:data_length]).unsqueeze(1)
        self.tau =  torch.cat([self.tau_1, self.tau_2, self.tau_3], dim = 1)                                                                                                                # shape: batch_size x 3
         
    def __len__(self):
        return len(self.theta)
    
    def __getitem__(self, index):
        return {
                #"t": self.t[index : index + 1, :],
                #"theta": self.theta[index : index + 1, :],
                #"theta_dot": self.theta_dot[index : index + 1, :],
                #"theta_Ddot": self.theta_Ddot[index : index + 1, :],
                #"s": self.s[index : index + 1, :],
                #"s_Ddot": self.s_Ddot[index : index + 1, :],
                #"tau": self.tau[index : index + 1, :],
                
                "theta": self.theta[index],
                "theta_dot": self.theta_dot[index],
                "theta_Ddot": self.theta_Ddot[index],
                "s": self.s[index],
                "s_Ddot": self.s_Ddot[index],
                "tau": self.tau[index],
                
                }
        
    
        