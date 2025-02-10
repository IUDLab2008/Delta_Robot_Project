import sys
import torch
import torch.nn as nn
import torch.optim as optim
import torch.autograd.functional as F  # Import functional differentiation utilities
from typing import Tuple

sys.path.append('/model')
from NNv3 import T_NNs, V_NNs, B_NNs

#---Declare global variables for dynamic system dimensions---#
BASE_RADIUS = 0.06
END_EFFECTOR_RADIUS = 0.045
LOWER_ARM_LENGTH = 0.176
GRAVI_ACCEL = 9.8
ALPHA = torch.deg2rad(torch.tensor([-30, 90, 210], dtype=torch.float32))


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

        self.T_NN = T_NNs()
        self.V_NN = V_NNs()
        self.B_NN = B_NNs(self.device)

    def lagrangian(self, theta_dot: torch.Tensor, s_dot: torch.Tensor, theta: torch.Tensor, z: torch.Tensor) -> torch.Tensor:

        return self.T_NN(theta_dot, s_dot).to(self.device).to(self.dtype) + self.V_NN(theta, z).to(self.device).to(self.dtype)


    def Lagrangian_Function(self, theta_dot: torch.Tensor, s_dot: torch.Tensor, theta: torch.Tensor, z: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        batch_size = theta_dot.shape[0]
        
        # Initialize tensors to store results for each batch
        ddLagrangian_dThetaDotdThetaDot_batched = []
        dLagragian_dTheta_batched = []
        ddLagrangian_dThetadThetaDot_batched = []
        
        # Process each batch element individually
        for i in range(batch_size):
            # Extract individual tensors for this batch
            theta_dot_i = theta_dot[i]
            s_dot_i = s_dot[i]
            theta_i = theta[i]
            z_i = z[i]
            
            # Compute Hessian for this batch element
            ddLagrangian_dThetaDotdThetaDot = F.hessian(
                lambda x: self.lagrangian(
                    x.unsqueeze(0), 
                    s_dot_i.unsqueeze(0), 
                    theta_i.unsqueeze(0), 
                    z_i.unsqueeze(0)
                )[0],  # Take first (only) element after unsqueezing
                theta_dot_i
            )
            
            # Compute Jacobian with respect to theta
            dLagragian_dTheta = F.jacobian(
                lambda x: self.lagrangian(
                    theta_dot_i.unsqueeze(0),
                    s_dot_i.unsqueeze(0),
                    x.unsqueeze(0),
                    z_i.unsqueeze(0)
                )[0],
                theta_i
            )
            
            # Compute mixed derivative
            ddLagrangian_dThetadThetaDot = F.jacobian(
                lambda x: dLagragian_dTheta,
                theta_dot_i
            )
            
            # Store results for this batch
            ddLagrangian_dThetaDotdThetaDot_batched.append(ddLagrangian_dThetaDotdThetaDot)
            dLagragian_dTheta_batched.append(dLagragian_dTheta)
            ddLagrangian_dThetadThetaDot_batched.append(ddLagrangian_dThetadThetaDot)
        
        # Stack results along batch dimension
        ddLagrangian_dThetaDotdThetaDot = torch.stack(ddLagrangian_dThetaDotdThetaDot_batched)
        dLagragian_dTheta = torch.stack(dLagragian_dTheta_batched)
        ddLagrangian_dThetadThetaDot = torch.stack(ddLagrangian_dThetadThetaDot_batched)
        
        return ddLagrangian_dThetaDotdThetaDot, dLagragian_dTheta, ddLagrangian_dThetadThetaDot

    def forward(self, theta_dot: torch.Tensor, s_dot: torch.Tensor, theta: torch.Tensor, z: torch.Tensor, s: torch.Tensor, s_Ddot: torch.Tensor, tau: torch.Tensor) -> torch.Tensor:
        ddLagrangian_dThetaDotdThetaDot, dLagragian_dTheta, ddLagrangian_dThetadThetaDot = self.Lagrangian_Function(
            theta_dot, s_dot, theta, z
        )
        
        # Assuming theta_dot needs to be unsqueezed for proper broadcasting
        theta_dot_expanded = theta_dot.unsqueeze(-1)
        
        print("ddLagrangian_dThetaDotdThetaDot shape:", ddLagrangian_dThetaDotdThetaDot.shape)
        print("tau shape: ", tau.unsqueeze(-1).shape)
        print("dLagragian_dTheta shape: ", dLagragian_dTheta.shape)
        print("ddLagrangian_dThetadThetaDot shape: ", ddLagrangian_dThetadThetaDot.shape)
        print("theta_dot_expanded shape: ", theta_dot_expanded.shape)
        
        # Compute the result for each batch
        result = torch.linalg.inv(ddLagrangian_dThetaDotdThetaDot) @ (
            tau.unsqueeze(-1) + self.B_NN(theta, s, s_Ddot).to(self.device).to(self.dtype) + 
            dLagragian_dTheta - 
            ddLagrangian_dThetadThetaDot @ theta_dot_expanded
        )
        
        print("result shape:", result.shape)
        
        return result
