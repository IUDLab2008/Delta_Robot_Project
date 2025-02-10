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
        
        Lagrangian = self.Lagrangian_Function(theta_dot, s_dot, theta, z).requires_grad_(True)
        
        dLagrangian_dThetaDot = self.compute_dLagrangian_dThetaDot(theta, theta_dot, s_dot, z)
        
        dLagrangian_dTheta = self.compute_dLagrangian_dTheta(theta, theta_dot, s_dot, z)
        
        ddLagrangian_dThetaDotdThetaDot = torch.autograd.functional.jacobian(
            lambda theta_dot: self.compute_dLagrangian_dThetaDot(theta, theta_dot, s_dot, z),
            theta_dot
        )
        
        ddLagrangian_dThetadThetaDot = torch.autograd.functional.jacobian(
            lambda theta_dot: self.compute_dLagrangian_dTheta(theta, theta_dot, s_dot, z),
            theta_dot
        )
        
        print("dLagrangian_dTheta shape: ", dLagrangian_dTheta.shape)
        print("ddLagrangian_dThetadThetaDot shape: ", ddLagrangian_dThetadThetaDot.shape)
        print("ddLagrangian_dThetaDotdThetaDot shape: ", ddLagrangian_dThetaDotdThetaDot.shape)
        
        return ddLagrangian_dThetaDotdThetaDot.unsqueeze(-1), dLagrangian_dTheta.unsqueeze(-1), ddLagrangian_dThetadThetaDot.unsqueeze(-1)

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
        
        print("result shape:", result.shape)
        
        return result