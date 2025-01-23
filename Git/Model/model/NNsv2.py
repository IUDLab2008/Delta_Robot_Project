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

class T_NNs(nn.Module):
    """
        Neural Network for predicting the total Kinetic Energy T(theta_dot, s_dot, t).

      Args:
          device (str): Device to run the model on (e.g., 'cpu' or 'cuda').
          input_dim (int): Dimension of the input tensor. Default is 7.
          output_dim (int): Dimension of the output tensor. Default is 1.
          hidden_layer_dim (int): Number of neurons in each hidden layer. Default is 128.
          num_hidden_layer (int): Number of hidden layers. Default is 5.
          activation (list): List of activation functions for each hidden layer. 

      Input:
          theta_dot (torch.Tensor): Tensor of joint velocities, shape (batch_size, 3).
          s_dot (torch.Tensor): Tensor of end-effector velocities, shape (batch_size, 3).
          t (torch.Tensor): Tensor of time, shape (batch_size, 1).

      Output:
          torch.Tensor: Predicted total Kinetic Energy, shape (batch_size, 1).
    """
    def __init__(
      self, 
      device: str, 
      input_dim: int = 7, 
      output_dim: int = 1, 
      hidden_layer_dim: int = 128, 
      num_hidden_layer: int = 5, 
      activation : list = 'tanh'
      ):

        super(T_NNs, self).__init__()
        
        self.device = device
        self.epoch = 0

        # Define the hidden layers
        self.L_d_layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        for _ in range(num_hidden_layer - 1):
            self.L_d_layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))
        
        # Add the final output layer
        self.L_d_layers.append(nn.Linear(hidden_layer_dim, output_dim))

        # Choose activation function for hidden layers
        if activation == 'sin':
          self.activation = torch.sin
        elif activation == 'tanh':
          self.activation = torch.tanh
        elif activation == 'relu':
          self.activation = torch.nn.ReLU()
        else:
          self.activation = torch.nn.LeakyReLU()

        # Softplus for the last activation layer
        self.last_activation = nn.Softplus()

    def forward(
                self, 
                theta_dot: torch.Tensor, 
                s_dot: torch.Tensor, 
                t: torch.Tensor
                ) -> torch.Tensor:
        input = torch.cat((theta_dot, s_dot, t), dim = 1)
        batch_size = input.shape[0]
        
        out_1 = input
        for layer in self.L_d_layers[:-1]:
            out_1 = self.activation(layer(out_1)) if isinstance(self.activation, nn.Module) else self.activation(layer(out_1))
        
        # Apply the final layer and Softplus activation
        out_1 = self.last_activation(self.L_d_layers[-1](out_1))
                        
        return out_1
    
    
#---Simple NNs for Potential Energy matrix according to the publication---#
class V_NNs(nn.Module):
    """
      Neural Network for predicting the total Kinetic Energy T(theta_dot, s_dot, t).

    Args:
        device (str): Device to run the model on (e.g., 'cpu' or 'cuda').
        input_dim (int): Dimension of the input tensor. Default is 7.
        output_dim (int): Dimension of the output tensor. Default is 1.
        hidden_layer_dim (int): Number of neurons in each hidden layer. Default is 128.
        num_hidden_layer (int): Number of hidden layers. Default is 5.
        activation (list): List of activation functions for each hidden layer. 

    Input:
        theta (torch.Tensor): Tensor of joint angle, shape (batch_size, 3).
        z (torch.Tensor): Tensor of end-effector Z coordinate, shape (batch_size, 1).
        
    Output:
        torch.Tensor: Predicted total Potential Energy, shape (batch_size, 1).
    """
    def __init__(
        self, 
        device: str, 
        input_dim: int = 4, 
        output_dim: int = 3, 
        hidden_layer_dim: int = 128, 
        num_hidden_layer: int = 5, 
        activation: list = 'tanh'
        ):

        super(V_NNs, self).__init__()
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

    def forward(
                self, 
                theta: torch.Tensor, 
                z: torch.Tensor
                ) -> torch.Tensor:
        input = torch.cat((theta, z), dim = 1)
        batch_size = input.shape[0]

        out = input
        for layer in self.layers[:-1]:
            out = self.activation(layer(out))
        out = self.layers[-1](out).to(self.device)
        return out
