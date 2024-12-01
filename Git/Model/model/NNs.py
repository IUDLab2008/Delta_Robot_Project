"""
Module Name: Neural Networks

Purpose:
  This module provides functions to initialize a simple Neural Network with given 
  parameters and process a forward pass of the Network        

Parameters:
  input_dim: dimension of input
  hidden_layer_dim: dimension of a hidden layer (assume all layers possess the same dimension)
  hidden_layer_num: number of hidden layers
  output_dim: dimension of output
  activation: activation of the Neural Network
"""

import torch
import torch.nn as nn
import torch.optim as optim

torch.set_default_dtype(torch.float32)

device = 'cuda' if torch.cuda.is_available() else 'cpu'

class NNs(nn.Module):
    def __init__(self, input_dim = 1, hidden_layer_dim = 1, hidden_layer_num = 1, output_dim = 1, activation = "tanh"):
        super(NNs, self).__init__()
        
        self.layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        
        for _ in range(hidden_layer_num - 1):
            self.layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))
        self.layers.append(nn.Linear(hidden_layer_dim, output_dim))
        
        self.epoch = 0
        
        if activation == 'sin':
          self.activation = torch.sin
        elif activation == 'tanh':
          self.activation = torch.tanh
        elif activation == 'relu':
          self.activation = torch.nn.ReLU()
        else:
          self.activation = torch.nn.LeakyReLU()    
          
    def forward(self, x):
        out = x
        for layer in self.layers[:-1]:
            out = self.activation(layer(out))
        out = self.layers[-1](out)
        return out
        
        