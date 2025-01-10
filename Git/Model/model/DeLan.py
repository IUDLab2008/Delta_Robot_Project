import sys
import torch
import torch.nn as nn
import torch.optim as optim

sys.path.append('/model')
from NNs import M_NNs, G_NNs, B_NNs


#---Declare global variable for dynamic system dimensions---#
f = 0.06
e = 0.045
l_1 = 0.176
g = 9.8
alpha = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))

#---Declare device---#
device='cuda' if torch.cuda.is_available() else 'cpu'


#---Calculation of Loss function---#
"""
Module Name: Loss Function
Description:
    This module produce the loss function of the PINNs
"""
class Dynamic_System(nn.Module):
    def __init__(self, device, dtype = torch.float32):
        super(Dynamic_System, self).__init__()
        self.dtype = dtype
        
        self.f_v1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_v2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_v3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        
        self.f_c1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_c2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_c3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype)) 
        
        self.Mass_NNs = M_NNs(device).to(dtype)
        self.Gravity_NNs = G_NNs(device).to(dtype)
        self.KBA_Term = B_NNs(device).to(dtype)
        
        
        #-----Define parameters for the whole system which consists of M, B, G parameters and parameters like f_v and f_c-----#
        sub_net_params = []
        sub_net_params.extend(list(self.Mass_NNs.parameters()))
        sub_net_params.extend(list(self.Gravity_NNs.parameters()))
        sub_net_params.extend(list(self.KBA_Term.parameters()))

        # Convert sub-network parameters to be part of Dynamic_System parameters
        for i, param in enumerate(sub_net_params):
            self.register_parameter(f'sub_net_param_{i}', param)
                
                
    def Auxiliary_Calculation(self, q, s, s_Ddot):
        """
        Extract parts of input tensor
        """

        M = self.Mass_NNs(q)
        G = self.Gravity_NNs(q)
        KAB = self.KBA_Term(q, s, s_Ddot)
        return M, G, KAB
    
    def forward(self, q, q_dot, s, s_Ddot, tau):
        q = q.to(self.dtype)
        q_dot = q_dot.to(self.dtype)
        s = s.to(self.dtype)
        s_Ddot = s_Ddot.to(self.dtype)
        tau = tau.to(self.dtype)
        
        batch_size = q.shape[0]        
        M, G, KAB = self.Auxiliary_Calculation(q, s, s_Ddot)
        
        #-----Create friction matrices for F_v and F_c with shape [3, 3]----#
        self.F_v = torch.zeros(3, 3, device = device, dtype = self.dtype)
        self.F_v[0, 0] = self.f_v1
        self.F_v[1, 1] = self.f_v2
        self.F_v[2, 2] = self.f_v3        
        
        self.F_c = torch.zeros(3, 3, device = device, dtype = self.dtype)
        self.F_c[0, 0] = self.f_c1
        self.F_c[1, 1] = self.f_c2
        self.F_c[2, 2] = self.f_c3
        
        self.F_v = self.F_v.expand(batch_size, 3, 3)
        self.F_c = self.F_c.expand(batch_size, 3, 3)

        q_Ddot = torch.linalg.inv(M) @ ( 
                - G + KAB - self.F_v @ (q_dot.unsqueeze(-1)) - self.F_c @ torch.sign(q_dot.unsqueeze(-1)) + tau.unsqueeze(-1)
                )
        return q_Ddot.squeeze(-1)   

                
