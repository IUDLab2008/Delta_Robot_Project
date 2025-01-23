import sys
import torch
import torch.nn as nn
import torch.optim as optim
from typing import Tuple

sys.path.append('/model')
from NNsv2 import T_NNs, V_NNs


#---Declare global variable for dynamic system dimensions---#
BASE_RADIUS = 0.06
END_EFFECTOR_RADIUS = 0.045
LOWER_ARM_LENGTH = 0.176
GRAVI_ACCEL = 9.8
ALPHA = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))


def T_Derivative(
    T_NNsInstance: nn.Module,
    theta_dot: torch.Tensor,
    s_dot: torch.Tensor,
    t: torch.Tensor, 
    device: str
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
        Summary: This modules calculate the first-order derivative of the Total Kinetic Energy w.r.t input elements (theta_dot, s_dot, t)
        Args:
            T_NNsInstance: An instance of Total Kinetic Energy Neural Network (T_NNs) with a forward method
            theta_dot: Tensor, input data of recorded angular velocities (shape: [batch_size, 3])
            s_dot: Tensor, input data of recorded End-effector Cartesian velocities (shape: [batch_size, 3])
            device: String, device to perform computation
        Output:
          Tuple containing:
            T_NN: Total Kinetic Energy, result of the forward pass through T_NNs    (shape: [batch_size, 1])
            ddT_dtdThetaDot: Tensor of First-order Derivative of Total Kinetic Energy w.r.t theta_dot (shape: [batch_size, 3, 1])
            ddT_dtdXDot: Tensor of First-order Derivative of Total Kinetic Energy w.r.t x_dot (shape: [batch_size, 3])
            ddT_dtdYDot: Tensor of First-order Derivative of Total Kinetic Energy w.r.t y_dot (shape: [batch_size, 3])
            ddT_dtdZDot: Tensor of First-order Derivative of Total Kinetic Energy w.r.t z_dot (shape: [batch_size, 3])
    """
    s_dot = s_dot.to(device).requires_grad_(True)
    theta_dot = theta_dot.to(device).requires_grad_(True)
    t = t.to(device).requires_grad_(True)
    
    T_NN = T_NNsInstance(theta_dot, s_dot, t)
    
    dT_dThetaDot = torch.autograd.grad(
        outputs = T_NN, inputs = theta_dot,
        grad_outputs=torch.ones_like(T_NN),
        create_graph=True, retain_graph = True
    )[0]
    
    dT_dSDot = torch.autograd.grad(
        outputs = T_NN, inputs = s_dot,
        grad_outputs=torch.ones_like(T_NN),
        create_graph=True, retain_graph = True
    )[0]
    
    ddT_dtdThetaDot = torch.autograd.grad(
        outputs = dT_dThetaDot, inputs = t,
        grad_outputs=torch.ones_like(dT_dThetaDot),
        create_graph=True, retain_graph = True
    )[0]
    
    ddT_dtdSDot = []
    for i in range(3):  # Loop over x_dot, y_dot, z_dot
        ddT_dtdSDot_i = torch.autograd.grad(
            outputs=dT_dSDot[:, i], inputs=t,
            grad_outputs=torch.ones_like(dT_dSDot[:, i]),
            create_graph=True, retain_graph=True
        )[0]
        ddT_dtdSDot.append(ddT_dtdSDot_i)
        
    ddT_dtdSDot = torch.stack(ddT_dtdSDot, dim=1)
    
    ddT_dtdXDot = ddT_dtdSDot[:, 0]
    ddT_dtdYDot = ddT_dtdSDot[:, 1]
    ddT_dtdZDot = ddT_dtdSDot[:, 2]
    
    return T_NN, ddT_dtdThetaDot.unsqueeze(-1), ddT_dtdXDot, ddT_dtdYDot, ddT_dtdZDot
    
def V_Derivative(
    V_NNsInstance: nn.Module, 
    theta: torch.Tensor, 
    z: torch.Tensor, 
    device: str
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """
        Summary: This modules calculate the first-order derivative of the Total Potential Energy w.r.t input elements (theta, z)
        Args:
            V_NNsInstance: An instance of Total Potential Energy Neural Network (V_NNs) with a forward method
            theta: Tensor, input data of recorded angles (shape: [batch_size, 3])
            z: Tensor, input data of recorded End-effector Cartesian Z-coordinate (shape: [batch_size, 1])
            device: String, device to perform computation
        Output:
            V_NN: Total Potential Energy, result of the forward pass through V_NNs    (shape: [batch_size, 1])
            dV_dTheta: Tensor of First-order Derivative of Total Potential Energy w.r.t theta (shape: [batch_size, 3, 1])
            dV_dZ: Tensor of First-order Derivative of Total Kinetic Energy w.r.t z (shape: [batch_size, 1])
    """
    z = z.to(device).requires_grad_(True)
    theta = theta.to(device).requires_grad_(True)
    
    V_NN = V_NNsInstance(theta, z)
    
    dV_dTheta = torch.autograd.grad(
        outputs = V_NN, inputs = theta,
        grad_outputs=torch.ones_like(V_NN),
        create_graph=True, retain_graph = True
    )[0]
    
    dV_dZ = torch.autograd.grad(
        outputs = V_NN, inputs = z,
        grad_outputs=torch.ones_like(V_NN),
        create_graph=True, retain_graph = True
    )[0]
    
    return V_NN, dV_dTheta.unsqueeze(-1), dV_dZ



class Dynamic_System(nn.Module):
    """
        Module Name: Dynamic_System
        Description: This module integrates the above Neural Networks in the sense of function as described in Publication
                    to predict the current torques acting on 3 motors
        Args:
            q: Recorded angular values at time t
            q_dot: Recorded angular velocities at time t
            s: Recorded absolute Cartesian Coordinate of the End-Effector at time t
            s_dot: Recorded absolute Cartesian Velocities of the End-Effector at time t
            t: Current time
        Output:
            tau_pred: Predicted torques acting on 3 motors at time t 
    """
    def __init__(self, device: str, dtype: torch.dtype = torch.float32):
        super(Dynamic_System, self).__init__()
        self.dtype = dtype
        self.device = device
        
        self.f_v1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_v2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_v3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        
        self.f_c1 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_c2 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype))
        self.f_c3 = nn.Parameter(torch.tensor([0.05], requires_grad=True, device=device, dtype = dtype)) 
        
        self.T_NN = T_NNs(device).to(dtype)
        self.V_NN = V_NNs(device).to(dtype)        
        
        #-----Define parameters for the whole system which consists of M, B, G parameters and parameters like f_v and f_c-----#
        sub_net_params = []
        sub_net_params.extend(list(self.T_NN.parameters()))
        sub_net_params.extend(list(self.V_NN.parameters()))
                
                
    def Auxiliary_Calculation(
        self, 
        q: torch.Tensor, 
        q_dot: torch.Tensor, 
        s: torch.Tensor, 
        s_dot: torch.Tensor, 
        t: torch.Tensor
        ) -> torch.Tensor:
        
        z = s_dot[:, 2:3]
        _, ddT_dtdThetaDot, ddT_dtdXDot, ddT_dtdYDot, ddT_dtdZDot = T_Derivative(self.T_NN, q_dot, s_dot, t, self.device)
        _, dV_dTheta, dV_dZ = V_Derivative(self.V_NN, q, z, self.device)
        
        #---Define K matrix---#
        K11 = (s[:, 0] * torch.cos((ALPHA[0])) + s[:, 1] * torch.sin((ALPHA[0])) + BASE_RADIUS - END_EFFECTOR_RADIUS) * torch.sin(q[:, 0]) - s[:, 2] * torch.cos(q[:, 0])
        K22 = (s[:, 0] * torch.cos((ALPHA[1])) + s[:, 1] * torch.sin((ALPHA[1])) + BASE_RADIUS - END_EFFECTOR_RADIUS) * torch.sin(q[:, 1]) - s[:, 2] * torch.cos(q[:, 1])
        K33 = (s[:, 0] * torch.cos((ALPHA[2])) + s[:, 1] * torch.sin((ALPHA[2])) + BASE_RADIUS - END_EFFECTOR_RADIUS) * torch.sin(q[:, 2]) - s[:, 2] * torch.cos(q[:, 2])
        
        K = torch.zeros((q.shape[0], 3, 3), device= self.device)                                                            
        K[:, 0, 0] = K11
        K[:, 1, 1] = K22
        K[:, 2, 2] = K33
        
        #---Define A matrix---#
        A = torch.zeros((q.shape[0], 3, 3), device= self.device)                                                             # batch_size x 3 x 3
        
        A[:, 0, 0] = s[:, 0] + END_EFFECTOR_RADIUS * torch.cos((ALPHA[0])) - BASE_RADIUS * torch.cos((ALPHA[0])) - LOWER_ARM_LENGTH * torch.cos((ALPHA[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 0, 1] = s[:, 0] + END_EFFECTOR_RADIUS * torch.cos((ALPHA[1])) - BASE_RADIUS * torch.cos((ALPHA[1])) - LOWER_ARM_LENGTH * torch.cos((ALPHA[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 0, 2] = s[:, 0] + END_EFFECTOR_RADIUS * torch.cos((ALPHA[2])) - BASE_RADIUS * torch.cos((ALPHA[2])) - LOWER_ARM_LENGTH * torch.cos((ALPHA[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 1, 0] = s[:, 1] + END_EFFECTOR_RADIUS * torch.sin((ALPHA[0])) - BASE_RADIUS * torch.sin((ALPHA[0])) - LOWER_ARM_LENGTH * torch.sin((ALPHA[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 1, 1] = s[:, 1] + END_EFFECTOR_RADIUS * torch.sin((ALPHA[1])) - BASE_RADIUS * torch.sin((ALPHA[1])) - LOWER_ARM_LENGTH * torch.sin((ALPHA[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 1, 2] = s[:, 1] + END_EFFECTOR_RADIUS * torch.sin((ALPHA[2])) - BASE_RADIUS * torch.sin((ALPHA[2])) - LOWER_ARM_LENGTH * torch.sin((ALPHA[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 2, 0] = s[:, 2] - LOWER_ARM_LENGTH * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 2, 1] = s[:, 2] - LOWER_ARM_LENGTH * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 2, 2] = s[:, 2] - LOWER_ARM_LENGTH * torch.cos(q[:, 2]) # shape: batch_size
        
        #---Define B matrix---#
        B = torch.zeros((q.shape[0], 3, 1), device = self.device)
        
        B[:, 0, 0] = ddT_dtdXDot.squeeze()
        B[:, 1, 0] = ddT_dtdYDot.squeeze()
        B[:, 2, 0] = (ddT_dtdZDot + dV_dZ).squeeze()
        
        #---KAB matrix---#
        KAB = K @ torch.linalg.inv(A) @ B
        
        tau = ddT_dtdThetaDot + dV_dTheta - KAB
        return tau
    
    def forward(
        self, 
        q: torch.Tensor, 
        q_dot: torch.Tensor, 
        s: torch.Tensor, 
        s_dot: torch.Tensor, 
        t: torch.Tensor
        ) -> torch.Tensor:
        
        q = q.to(self.dtype)
        q_dot = q_dot.to(self.dtype)
        s = s.to(self.dtype)
        s_dot = s_dot.to(self.dtype)
        t = t.to(self.dtype)
        
        batch_size = t.shape[0]
        
        self.F_v = torch.zeros(3, 3, device = self.device, dtype = self.dtype)
        self.F_v[0, 0] = self.f_v1
        self.F_v[1, 1] = self.f_v2
        self.F_v[2, 2] = self.f_v3        
        
        self.F_c = torch.zeros(3, 3, device = self.device, dtype = self.dtype)
        self.F_c[0, 0] = self.f_c1
        self.F_c[1, 1] = self.f_c2
        self.F_c[2, 2] = self.f_c3
        
        self.F_v = self.F_v.expand(batch_size, 3, 3)
        self.F_c = self.F_c.expand(batch_size, 3, 3)

        #---Predict Torque---#
        tau_pred = self.Auxiliary_Calculation(q, q_dot, s, s_dot, t) + self.F_v @ (q_dot.unsqueeze(-1)) - self.F_c @ torch.sign(q_dot.unsqueeze(-1)) 
        return tau_pred.squeeze()   

                
