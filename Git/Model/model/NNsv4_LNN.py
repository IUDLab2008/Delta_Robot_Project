import torch
from torch.autograd import grad
import torch.nn as nn
import numpy as np
import math
import torch.optim as optim

torch.set_default_dtype(torch.float32)

#---Declare global variable for dynamic system dimensions---#
BASE_RADIUS = 0.06
END_EFFECTOR_RADIUS = 0.045
LOWER_ARM_LENGTH = 0.176
GRAVI_ACCEL = 9.8
ALPHA = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))



#---Special NNs for Mass matrix according to the publication---#
class LAGRANGIAN_NNs(nn.Module):
    """
        Neural Network for predicting the total Lagrangian function L(theta_dot, s_dot, theta, z).

      Args:
          input_dim (int): Dimension of the input tensor. Default is 7.
          output_dim (int): Dimension of the output tensor. Default is 1.
          hidden_layer_dim (int): Number of neurons in each hidden layer. Default is 128.
          num_hidden_layer (int): Number of hidden layers. Default is 5.
          activation (list): List of activation functions for each hidden layer. 

      Input:
          theta_dot (torch.Tensor): Tensor of joint velocities, shape (batch_size, 3).
          s_dot (torch.Tensor): Tensor of end-effector velocities, shape (batch_size, 3).
          theta (torch.Tensor): Tensor of joint angles, shape (batch_size, 3).
          z (torch.Tensor): Tensor of end-effector z-value, shape (batch_size, 1).

      Output:
          torch.Tensor: Predicted total Lagrangian function, shape (batch_size, 1).
    """
    def __init__(
      self, 
      input_dim: int = 10, 
      output_dim: int = 1, 
      hidden_layer_dim: int = 128, 
      num_hidden_layer: int = 5, 
      activation : list = 'tanh'
      ):

        super(LAGRANGIAN_NNs, self).__init__()
        
        self.epoch = 0

        # Define the hidden layers
        self.layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        for _ in range(num_hidden_layer - 1):
            self.layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))

        self.layers.append(nn.Linear(hidden_layer_dim, output_dim))

        # Choose activation function for hidden layers
        if activation == 'sin':
          self.activation = torch.sin
        elif activation == 'tanh':
          self.activation = torch.tanh
        elif activation == 'relu':
          self.activation = torch.nn.ReLU()
        else:
          self.activation = torch.nn.LeakyReLU()

    def forward(
                self, 
                theta_dot: torch.Tensor, 
                s_dot: torch.Tensor, 
                theta: torch.Tensor, 
                z: torch.Tensor
                ) -> torch.Tensor:
        input = torch.cat([theta_dot, s_dot, theta, z], dim = 1)
        
        out_1 = input
        for layer in self.layers[:-1]:
            out_1 = self.activation(layer(out_1)) if isinstance(self.activation, nn.Module) else self.activation(layer(out_1))
        
        out_1 = self.layers[-1](out_1)
        return out_1



#---Calculation of B Matrix---#
"""
Module Name: B_NNs
Description:
    This module predicts the elements of B(s)
    This module utilizes inputs "s" which yields the results of B
    The following calculations is used to compute the term K @ inv(A) @ B, which takes in "s_Ddot" and "q"

"""
class B_NNs(nn.Module):
    def __init__(self, device, input_dim = 3, output_dim = 3, hidden_layer_dim = 128, num_hidden_layer = 3, activation='tanh'):

        super(B_NNs, self).__init__()
        self.device = device
        
        # Define the hidden layers
        self.layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        for _ in range(num_hidden_layer - 1):
            self.layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))
        
        # Add the final output layer
        self.layers.append(nn.Linear(hidden_layer_dim, output_dim))

        self.epoch = 0

        # Choose activation function for hidden layers
        if activation == 'sin':
          self.activation = torch.sin
        elif activation == 'tanh':
          self.activation = torch.tanh
        elif activation == 'relu':
          self.activation = torch.nn.ReLU()
        else:
          self.activation = torch.nn.LeakyReLU()

    def forward(self, q, s, s_Ddot):        
        out = s_Ddot
        for layer in self.layers[:-1]:
            out = self.activation(layer(out))
        out = self.layers[-1](out).to(self.device)
        out = out.unsqueeze(-1)
                        
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
        
        return K @ torch.linalg.inv(A) @ out    # shape: batch_size x 3 x 1