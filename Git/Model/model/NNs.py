import torch
from torch.autograd import grad
import torch.nn as nn
import numpy as np
import math
import torch.optim as optim

torch.set_default_dtype(torch.float32)


#---Declare global variable for dynamic system dimensions---#
f = 0.06
e = 0.045
l_1 = 0.176
g = 9.8
alpha = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))



#---Special NNs for Mass matrix according to the publication---#
"""
Module Name: M_NNs
Description:
    This module predicts the structure of matrix M(q)
    M(q) is often referred as Mass/Inertia Matrix is a symmetric, positive definite matrix which follows Cholesky Descomposition
    Based on Cholesky Descomposition: M can be seen as L @ L.T with L is a lower triangular matrix, which is the combination of a diagonal matrix L_d and a strictly lower triangular matrix L_o
    This module utilizes the input "q" to predict L_d, L_o which then produce L and M    
"""

#---Thay vì phân ra thành 2 Network thì dùng 1 cái với output là 6, chỉ dùng activation Softplus cho layer cuối---#
class M_NNs(nn.Module):
    def __init__(self, device, input_dim = 3, output_dim = 6, hidden_layer_dim = 128, num_hidden_layer = 5, activation = 'tanh'):

        super(M_NNs, self).__init__()
        
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

    def forward(self, q):
        batch_size = q.shape[0]
        
        out_1 = q
        for layer in self.L_d_layers[:-1]:
            out_1 = self.activation(layer(out_1)) if isinstance(self.activation, nn.Module) else self.activation(layer(out_1))
        
        # Apply the final layer and Softplus activation
        out_1 = self.last_activation(self.L_d_layers[-1](out_1))
                        
        L = torch.zeros((batch_size, 3, 3), device = self.device)
        L [:, 0, 0] = out_1[:, 0]
        L [:, 1, 1] = out_1[:, 1]
        L [:, 2, 2] = out_1[:, 2]
        
        L [:, 1, 0] = out_1[:, 3]
        L [:, 2, 0] = out_1[:, 4]
        L [:, 2, 1] = out_1[:, 5]
        
        return L @ L.permute(0, 2, 1)
    
    
#---Simple NNs for Gravitational matrix according to the publication---#
"""
Module Name: G_NNs
Description:
    This module predicts the elements of G(q)
    This module utilizes inputs "cos(q)" which yields the results of G(q)
"""
class G_NNs(nn.Module):
    def __init__(self, device, input_dim = 3, output_dim = 3, hidden_layer_dim = 128, num_hidden_layer = 3, activation='tanh'):

        super(G_NNs, self).__init__()
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

    def forward(self, q):
        batch_size = q.shape[0]

        out = torch.cos(q)
        for layer in self.layers[:-1]:
            out = self.activation(layer(out))
        out = self.layers[-1](out).to(self.device)
        out = out.unsqueeze(-1)
        #out = torch.reshape(out, (batch_size, 3, 1))
        return out



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
        batch_size = s_Ddot.shape[0]
        
        out = s_Ddot
        for layer in self.layers[:-1]:
            out = self.activation(layer(out))
        out = self.layers[-1](out).to(self.device)
        out = out.unsqueeze(-1)
                
        #out = torch.reshape(out, (batch_size, 3, 1))
        
        #----------------Define K matrix---------------------#
        K11 = (s[:, 0] * torch.cos((alpha[0])) + s[:, 1] * torch.sin((alpha[0])) + f - e) * torch.sin(q[:, 0]) - s[:, 2] * torch.cos(q[:, 0])
        K22 = (s[:, 0] * torch.cos((alpha[1])) + s[:, 1] * torch.sin((alpha[1])) + f - e) * torch.sin(q[:, 1]) - s[:, 2] * torch.cos(q[:, 1])
        K33 = (s[:, 0] * torch.cos((alpha[2])) + s[:, 1] * torch.sin((alpha[2])) + f - e) * torch.sin(q[:, 2]) - s[:, 2] * torch.cos(q[:, 2])
        
        K = torch.zeros((q.shape[0], 3, 3), device= self.device)                                                            
        K[:, 0, 0] = K11
        K[:, 1, 1] = K22
        K[:, 2, 2] = K33
        
        #--------------------------------Define A matrix--------------------------------#
        A = torch.zeros((q.shape[0], 3, 3), device= self.device)                                                             # batch_size x 3 x 3
        
        A[:, 0, 0] = s[:, 0] + e * torch.cos((alpha[0])) - f * torch.cos((alpha[0])) - l_1 * torch.cos((alpha[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 0, 1] = s[:, 0] + e * torch.cos((alpha[1])) - f * torch.cos((alpha[1])) - l_1 * torch.cos((alpha[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 0, 2] = s[:, 0] + e * torch.cos((alpha[2])) - f * torch.cos((alpha[2])) - l_1 * torch.cos((alpha[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 1, 0] = s[:, 1] + e * torch.sin((alpha[0])) - f * torch.sin((alpha[0])) - l_1 * torch.sin((alpha[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 1, 1] = s[:, 1] + e * torch.sin((alpha[1])) - f * torch.sin((alpha[1])) - l_1 * torch.sin((alpha[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 1, 2] = s[:, 1] + e * torch.sin((alpha[2])) - f * torch.sin((alpha[2])) - l_1 * torch.sin((alpha[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 2, 0] = s[:, 2] - l_1 * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 2, 1] = s[:, 2] - l_1 * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 2, 2] = s[:, 2] - l_1 * torch.cos(q[:, 2]) # shape: batch_size
        
        #print('A shape:', A.shape)
        #print('K shape:', K.shape)
        #print('out shape:', out.shape)
        #print('multiplication shape:', (K @ torch.linalg.inv(A) @ out).shape)"""
        
        return K @ torch.linalg.inv(A) @ out    # shape: batch_size x 3 x 1
