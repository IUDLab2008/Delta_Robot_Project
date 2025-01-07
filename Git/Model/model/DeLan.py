import torch
import torch.nn as nn
import torch.optim as optim
import torch.autograd.grad as grad
from NNs import M_NNs, G_NNs, B_NNs


#---Declare global variable for dynamic system dimensions---#
f = 0.06
e = 0.045
l_1 = 0.176
g = 9.8
alpha = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))


#---Calculation of Loss function---#
"""
Module Name: Loss Function
Description:
    This module produce the loss function of the PINNs
    
"""
class Dynamic_System(nn.Module):
    def __init__(self, M_model, G_model, B_model):
        super(Dynamic_System, self).__init__()
        
        self.f_v1 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_v2 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_v3 = torch.tensor([0.05], requires_grad=True, device = device)
        
        self.f_c1 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_c2 = torch.tensor([0.05], requires_grad=True, device = device)
        self.f_c3 = torch.tensor([0.05], requires_grad=True, device = device) 
        
        self.Mass_NNs = M_NNs()
        self.Gravity_NNs = G_NNs()
        self.KBA_Term = B_NNs()
        
        #-----Create friction matrices for F_v and F_c with shape [3, 3]----#
        self.F_v = torch.zeros(3, 3)
        self.F_v[0, 0] = self.f_v1
        self.F_v[1, 1] = self.f_v2
        self.F_v[2, 2] = self.f_v3        
        
        self.F_c = torch.zeros(3, 3)
        self.F_c[0, 0] = self.f_c1
        self.F_c[1, 1] = self.f_c2
        self.F_c[2, 2] = self.f_c3
        
        #-----Define optimizer for the whole system which consists of M, B, G and parameters like f_v and f_c-----#
        self.optimizer = optim.Adam([
            {'params': self.Mass_NNs.parameters()},
            {'params': self.Gravity_NNs.parameters()},
            {'params': self.KBA_Term.parameters()},
            {'params': [self.f_v1]},
            {'params': [self.f_v2]},
            {'params': [self.f_v3]},
            {'params': [self.f_c1]},
            {'params': [self.f_c2]},
            {'params': [self.f_c3]},
        ], lr=1e-4)
                
    def Auxiliary_Calculation(self, q, s, s_Ddot):
        """
        Extract parts of input tensor
        """

        M = self.Mass_NNs(q)
        G = self.Gravity_NNs(torch.cos(q))
        KAB = self.KBA_Term(q, s, s_Ddot)
        
        return M, G, KAB
    
    def forward(self, q, q_dot, s, s_Ddot, tau):
        M, G, KAB = self.Auxiliary_Calculation(q, q_dot, s, s_Ddot)
        q_Ddot = torch.linalg.inv(M) @ (- G + KAB - self.F_v @ q_dot - self.F_c @ torch.sign(q_dot) + tau)
        return q_Ddot       
    
    def loss(self, q, q_dot, s, s_Ddot, tau, u_t_1):
        return torch.mean((self.forward(q, q_dot, s, s_Ddot, tau) - u_t_1)**2)
    
    """
    Sửa lại hàm train, hàm loss() bị sai args, chưa rút data từ train
    """
    def train(self, epochs, train_Data):
        q = train_Data['q']
        q_dot = train_Data['q_dot']
        s = train_Data['s']
        s_Ddot = train_Data['s_Ddot']
        tau = train_Data['tau']
        u_t_1 = train_Data['u_t_1']
        
        for epoch in range(epochs):
            self.optimizer.zero_grad()
            loss = self.loss(q, q_dot, s, s_Ddot, tau, u_t_1)
            loss.backward()
            self.optimizer.step()
            
            if epoch % 100 == 0:
                print(f"Epoch {epoch}, Loss: {loss.item()}, f_v1: {self.f_v1.item()}, f_v2: {self.f_v2.item()}, f_v3: {self.f_v3.item()}")
                print(f"f_c1: {self.f_c1.item()}, f_c2: {self.f_c2.item()}, f_c3: {self.f_c3.item()}")
                print("-------------------------------------")
                
