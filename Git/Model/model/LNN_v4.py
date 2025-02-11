import sys
import torch
import torch.nn as nn
import torch.optim as optim
import torch.autograd.functional as F  # Import functional differentiation utilities
from typing import Tuple

sys.path.append('/model')
from NNsv4_LNN import LAGRANGIAN_NNs, B_NNs


class Dynamic_System(nn.Module):
    def __init__(self, device: str, dtype: torch.dtype = torch.float32):
        super(Dynamic_System, self).__init__()
        self.dtype = dtype
        self.device = device

        self.f_v1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))
        self.f_v2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))
        self.f_v3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))

        self.f_c1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))
        self.f_c2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))
        self.f_c3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype=dtype))

        self.Lagrangian_Function = LAGRANGIAN_NNs().to(self.device)
        self.B_NN = B_NNs(self.device)
        
        sub_net_params = []
        sub_net_params.extend(list(self.B_NN.parameters()))
        sub_net_params.extend(list(self.Lagrangian_Function.parameters()))

    def compute_dLagrangian_dThetaDot(
            self, 
            theta, 
            theta_dot, 
            s_dot, 
            z
        ):

        Lagrangian = self.Lagrangian_Function(theta_dot, s_dot, theta, z).requires_grad_(True)

        dLagrangian_dThetaDot = torch.autograd.grad(
            outputs=Lagrangian,
            inputs=theta_dot,
            grad_outputs=torch.ones_like(Lagrangian),
            create_graph=True,
            retain_graph=True,
            allow_unused=False
        )[0]
        return dLagrangian_dThetaDot
    
    
    def compute_dLagrangian_dTheta(
            self, 
            theta, 
            theta_dot, 
            s_dot, 
            z
        ):

        Lagrangian = self.Lagrangian_Function(theta_dot, s_dot, theta, z).requires_grad_(True)

        dLagrangian_dTheta = torch.autograd.grad(
            outputs=Lagrangian,
            inputs=theta,
            grad_outputs=torch.ones_like(Lagrangian),
            create_graph=True,
            retain_graph=True,
            allow_unused=False
        )[0]
        
        return dLagrangian_dTheta


    def Compute_Derivatives(
            self, 
            theta_dot: torch.Tensor, 
            s_dot: torch.Tensor, 
            theta: torch.Tensor, 
            z: torch.Tensor
        ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        
        theta_dot = theta_dot.requires_grad_(True)
        theta = theta.requires_grad_(True)
        
        batch_size = theta_dot.shape[0]
        
        ddLagrangian_dThetaDotdThetaDot_batched = []
        dLagragian_dTheta_batched = []
        ddLagrangian_dThetadThetaDot_batched = []
        
        for i in range(batch_size):
            theta_dot_i = theta_dot[i].unsqueeze(0)
            s_dot_i = s_dot[i].unsqueeze(0)
            theta_i = theta[i].unsqueeze(0)
            z_i = z[i].unsqueeze(0)
        
            Lagrangian_i = self.Lagrangian_Function(theta_dot_i, s_dot_i, theta_i, z_i).requires_grad_(True)
            
            dLagrangian_dThetaDot_i = self.compute_dLagrangian_dThetaDot(theta_i, theta_dot_i, s_dot_i, z_i)
            
            dLagrangian_dTheta_i = self.compute_dLagrangian_dTheta(theta_i, theta_dot_i, s_dot_i, z_i).permute(1, 0)
            
            ddLagrangian_dThetaDotdThetaDot_i = torch.autograd.functional.jacobian(
                lambda theta_dot_i: self.compute_dLagrangian_dThetaDot(theta_i, theta_dot_i, s_dot_i, z_i),
                theta_dot_i
            ).squeeze()
            
            ddLagrangian_dThetadThetaDot_i = torch.autograd.functional.jacobian(
                lambda theta_dot_i: self.compute_dLagrangian_dTheta(theta_i, theta_dot_i, s_dot_i, z_i),
                theta_dot_i
            ).squeeze()
            
            ddLagrangian_dThetaDotdThetaDot_batched.append(ddLagrangian_dThetaDotdThetaDot_i)
            dLagragian_dTheta_batched.append(dLagrangian_dTheta_i)
            ddLagrangian_dThetadThetaDot_batched.append(ddLagrangian_dThetadThetaDot_i)
        
        ddLagrangian_dThetaDotdThetaDot = torch.stack(ddLagrangian_dThetaDotdThetaDot_batched)
        dLagrangian_dTheta = torch.stack(dLagragian_dTheta_batched)
        ddLagrangian_dThetadThetaDot = torch.stack(ddLagrangian_dThetadThetaDot_batched)
        

        
        return ddLagrangian_dThetaDotdThetaDot, dLagrangian_dTheta, ddLagrangian_dThetadThetaDot

    def forward(
            self, 
            theta_dot: torch.Tensor, 
            s_dot: torch.Tensor, 
            theta: torch.Tensor, 
            z: torch.Tensor, 
            s: torch.Tensor, 
            s_Ddot: torch.Tensor, 
            tau: torch.Tensor 
        ) -> torch.Tensor: 
        
        ddLagrangian_dThetaDotdThetaDot, dLagragian_dTheta, ddLagrangian_dThetadThetaDot = self.Compute_Derivatives(
            theta_dot, s_dot, theta, z
        )

        theta_dot_expanded = theta_dot.unsqueeze(-1)
        
        # Compute the result for each batch
        result = torch.linalg.inv(ddLagrangian_dThetaDotdThetaDot) @ (
            tau.unsqueeze(-1) + self.B_NN(theta, s, s_Ddot).to(self.device).to(self.dtype) + 
            dLagragian_dTheta - 
            ddLagrangian_dThetadThetaDot @ theta_dot_expanded
        )
                        
        return result