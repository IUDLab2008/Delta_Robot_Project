"""
Module name: Physics-Informed Neural Networks

Purpose:
  This module initializes physics parameters, unknown parametes;
  provides functions to compute Inverse Dynamics Equation of the 3-DoF RRR Delta robot
  along with computing residual loss, data loss and total loss based on MSELoss()
"""

import torch
import torch.nn as nn
import torch.optim as optim
import torch.autograd.grad as grad
from NNs import NNs

torch.set_default_dtype(torch.float32)

device = 'cuda' if torch.cuda.is_available() else 'cpu'

class PINNs(NNs):
    def __init__(self):
        super(self).__init__()
        
        #-------------Initialize base parameters-----------#
        self.f = 0.06
        self.e = 0.045
        self.l_1 = 0.176
        self.alpha = torch.deg2rad(torch.tensor([-30, 90, 210], device = device, dtype = torch.float32))
        
        self.g = 9.8
        
        #-------------Initialize learnable parameters--------------#
        self.l_1c = torch.tensor([0.01], requires_grad=True, device = device)
        self.m_0 = torch.tensor([0.01], requires_grad=True, device = device)
        self.m_2 = torch.tensor([0.01], requires_grad=True, device = device)
        self.m_1 = torch.tensor([0.01], requires_grad=True, device = device)
        self.I_1 = torch.tensor([0.01], requires_grad=True, device = device)
        
        self.f_v1 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_v2 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_v3 = torch.tensor([0.05], requires_grad=True, device = device)
        
        self.f_c1 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_c2 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_c3 = torch.tensor([0.05], requires_grad=True, device = device)
        
    #---------------Function for computing First and Second order Derivatives----------------#
    def Compute_Derivatives(self, t, x_pred):
        x_pred_grad = torch.autograd.grad(
            outputs=x_pred, inputs=t,
            grad_outputs=torch.ones_like(x_pred),
            create_graph=True
        )[0]
        
        x_pred_grad2 = torch.autograd.grad(
            outputs=x_pred_grad, inputs=t,
            grad_outputs=torch.ones_like(x_pred_grad),
            create_graph=True
        )[0]

        return x_pred_grad, x_pred_grad2
    
    #------------Compute the Inverse Dynamic equation, output the difference between predicted torque and ground-truth data---------------#
    def Inverse_Kinematic_eq(self):
        self.theta_1_pred = self.model(self.t)[:, 0:1]                                                  # shape: batch_size x 1
        self.theta_2_pred = self.model(self.t)[:, 1:2]                                                  # shape: batch_size x 1
        self.theta_3_pred = self.model(self.t)[:, 2:3]                                                  # shape: batch_size x 1
                
        
        theta_grad_1, theta_grad2_1 = self.compute_derivatives(self.t, self.theta_1_pred)               # shape: batch_size x 1
        theta_grad_2, theta_grad2_2 = self.compute_derivatives(self.t, self.theta_2_pred)               # shape: batch_size x 1
        theta_grad_3, theta_grad2_3 = self.compute_derivatives(self.t, self.theta_3_pred)               # shape: batch_size x 1
                
        theta_dot  = torch.cat([theta_grad_1, theta_grad_2, theta_grad_3], dim = 1)                     # shape: batch_size x 3
        theta_Ddot = torch.cat([theta_grad2_1, theta_grad2_2, theta_grad2_3], dim = 1)                  # shape: batch_size x 3
                
        #---------------Define M matrix----------------#
        M = torch.zeros(self.theta_1.shape[0], 3, 3)
        M[:, 0, 0] = 2 * self.I_1 + self.m_2 * (self.l_1 ** 2)
        M[:, 1, 1] = 2 * self.I_1 + self.m_2 * (self.l_1 ** 2)
        M[:, 2, 2] = 2 * self.I_1 + self.m_2 * (self.l_1 ** 2)
                
        #---------------Define G matrix-----------------#
        G = torch.zeros(self.theta_1.shape[0], 3, 1)
        G[:, 0, 0] = (self.m_1 * self.l_1c + self.m_2 * self.l_1) * self.g * torch.cos(self.theta_1_pred.squeeze(-1))             
        G[:, 1, 0] = (self.m_1 * self.l_1c + self.m_2 * self.l_1) * self.g * torch.cos(self.theta_2_pred.squeeze(-1))                                    
        G[:, 2, 0] = (self.m_1 * self.l_1c + self.m_2 * self.l_1) * self.g * torch.cos(self.theta_3_pred.squeeze(-1))                                     
                
        #---------------------------Define Friction matrices----------------------------#
        #-----Create friction matrices for F_v and F_c with shape [batch_size, 3, 3]----#
        F_v = torch.zeros(3, 3)
        F_v[0, 0] = self.f_v1
        F_v[1, 1] = self.f_v2
        F_v[2, 2] = self.f_v3
        
        F_v = F_v.unsqueeze(0).expand(self.theta_1.shape[0], -1, -1)                                               
        
        
        F_c = torch.zeros(3, 3)
        F_c[0, 0] = self.f_c1
        F_c[1, 1] = self.f_c2
        F_c[2, 2] = self.f_c3
        
        F_c = F_c.unsqueeze(0).expand(self.theta_1.shape[0], -1, -1) 
        
        
        #----------------Define K matrix---------------------#
        K11 = (self.s[:, 0] * torch.cos((self.alpha[0])) + self.s[:, 1] * torch.sin((self.alpha[0])) + self.f - self.e) * torch.sin(self.theta_1_pred.squeeze(-1)) - self.s[:, 2] * torch.cos(self.theta_1_pred.squeeze(-1))
        K22 = (self.s[:, 0] * torch.cos((self.alpha[1])) + self.s[:, 1] * torch.sin((self.alpha[1])) + self.f - self.e) * torch.sin(self.theta_2_pred.squeeze(-1)) - self.s[:, 2] * torch.cos(self.theta_2_pred.squeeze(-1))
        K33 = (self.s[:, 0] * torch.cos((self.alpha[2])) + self.s[:, 1] * torch.sin((self.alpha[2])) + self.f - self.e) * torch.sin(self.theta_3_pred.squeeze(-1)) - self.s[:, 2] * torch.cos(self.theta_3_pred.squeeze(-1))
        
        K = torch.zeros((self.theta_1_pred.shape[0], 3, 3))                                                            
        K[:, 0, 0] = K11
        K[:, 1, 1] = K22
        K[:, 2, 2] = K33
        
        #--------------------------------Define A matrix--------------------------------#
        A = torch.zeros((self.theta_1_pred.shape[0], 3, 3))                                                             # batch_size x 3 x 3
        
        A[:, 0, 0] = self.s[:, 0] + self.e * torch.cos((self.alpha[0])) - self.f * torch.cos((self.alpha[0])) - self.l_1 * torch.cos((self.alpha[0])) * torch.cos(self.theta_1_pred.squeeze(-1)) # shape: batch_size
        A[:, 0, 1] = self.s[:, 0] + self.e * torch.cos((self.alpha[1])) - self.f * torch.cos((self.alpha[1])) - self.l_1 * torch.cos((self.alpha[1])) * torch.cos(self.theta_2_pred.squeeze(-1)) # shape: batch_size
        A[:, 0, 2] = self.s[:, 0] + self.e * torch.cos((self.alpha[2])) - self.f * torch.cos((self.alpha[2])) - self.l_1 * torch.cos((self.alpha[2])) * torch.cos(self.theta_3_pred.squeeze(-1)) # shape: batch_size

        A[:, 1, 0] = self.s[:, 1] + self.e * torch.sin((self.alpha[0])) - self.f * torch.sin((self.alpha[0])) - self.l_1 * torch.sin((self.alpha[0])) * torch.cos(self.theta_1_pred.squeeze(-1)) # shape: batch_size
        A[:, 1, 1] = self.s[:, 1] + self.e * torch.sin((self.alpha[1])) - self.f * torch.sin((self.alpha[1])) - self.l_1 * torch.sin((self.alpha[1])) * torch.cos(self.theta_2_pred.squeeze(-1)) # shape: batch_size
        A[:, 1, 2] = self.s[:, 1] + self.e * torch.sin((self.alpha[2])) - self.f * torch.sin((self.alpha[2])) - self.l_1 * torch.sin((self.alpha[2])) * torch.cos(self.theta_3_pred.squeeze(-1)) # shape: batch_size

        A[:, 2, 0] = self.s[:, 2] - self.l_1 * torch.cos(self.theta_1_pred.squeeze(-1)) # shape: batch_size
        A[:, 2, 1] = self.s[:, 2] - self.l_1 * torch.cos(self.theta_2_pred.squeeze(-1)) # shape: batch_size
        A[:, 2, 2] = self.s[:, 2] - self.l_1 * torch.cos(self.theta_3_pred.squeeze(-1)) # shape: batch_size
        
        
        #------------Define B matrix-----------#
        B = torch.zeros((self.theta_1.shape[0], 3, 1))                                                            # batch_size x 3 x 1
        
        B[:, 0, 0] = (self.m_0 + 3 * self.m_2) * self.s_Ddot[:, 0]
        B[:, 1, 0] = (self.m_0 + 3 * self.m_2) * self.s_Ddot[:, 1]
        B[:, 2, 0] = (self.m_0 + 3 * self.m_2) * (self.s_Ddot[:, 2] - self.g)
        
        equation = torch.matmul(M, theta_Ddot.unsqueeze(-1)) + G - torch.matmul(torch.matmul(K, torch.linalg.inv(A)), B) + torch.matmul(F_v, (theta_dot.unsqueeze(-1))) + torch.matmul(F_c, (torch.sign(theta_dot.unsqueeze(-1)))) - self.tau.unsqueeze(-1)
        equation = equation.squeeze(-1)

        return equation
    
    def residual_loss(self):
        equation = self.Inverse_Dynamic_eq()
        return torch.mean(equation[:, 0:1] ** 2) + torch.mean(equation[:, 1:2] ** 2) + torch.mean(equation[:, 2:3] ** 2)
    
    def data_loss(self):
        
        return torch.mean((self.theta_1_pred - self.theta_1) ** 2) + torch.mean((self.theta_2_pred - self.theta_2) ** 2) + torch.mean((self.theta_3_pred - self.theta_3) ** 2)