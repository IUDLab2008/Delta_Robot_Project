import torch
from torch.autograd import grad
import torch.nn as nn
import numpy as np
import math
import torch.optim as optim

import h5py
import scipy.io

torch.set_default_dtype(torch.float32)

device = 'cpu' if torch.cuda.is_available() else 'cpu'


#---Declare global variable for dynamic system dimensions---#
f = 0.06
e = 0.045
l_1 = 0.176
g = 9.8
alpha = torch.deg2rad(torch.tensor([-30, 90, 210], dtype = torch.float32))


#---Define customized RK4 method---#
def RK4_method(q, q_dot, dynamic_equation, time_step):
    """
        Summary: Perform a single RK4 step to predict q, q_dot for the next time-step
        Input:
            q: Recorded angles data at time t_k
            q_dot: Recorded angular velocities at time t_k
            dynamic_equation: Resulted angular accelerations at time t_k by applying Euler-Lagrangian equation
            time_step: Fixed time-step size
        Output:
            6x1 State at time t_k+1 tensor x = [q_t+1, q_dot_t+1] 
    """

    k_1q = time_step * q_dot
    k_2q = time_step * (q_dot + k_1q / 2)
    k_3q = time_step * (q_dot + k_2q / 2)
    k_4q = time_step * (q_dot + k_3q)
    
    q_k1 = q + (k_1q + 2 * k_2q + 2 * k_3q + k_4q) / 6
    
    k_1qdot = time_step * dynamic_equation(q, q_dot)
    k_2qdot = time_step * dynamic_equation(q + k_1q / 2, q_dot + k_1qdot / 2)
    k_3qdot = time_step * dynamic_equation(q + k_2q / 2, q_dot + k_2qdot / 2)
    k_4qdot = time_step * dynamic_equation(q + k_3q, q_dot + k_3qdot)
    
    q_dot_k1 = q_dot + (k_1qdot + 2 * k_2qdot + 2 * k_3qdot + k_4qdot) / 6
    
    return q_k1, q_dot_k1


#---Define forward and inverse angular transformation---#




#---Special NNs for Mass matrix according to the publication---#
"""
Module Name: M_NNs
Description:
    This module predicts the structure of matrix M(q)
    M(q) is often referred as Mass/Inertia Matrix is a symmetric, positive definite matrix which follows Cholesky Descomposition
    Based on Cholesky Descomposition: M can be seen as L @ L.T with L is a lower triangular matrix, which is the combination of a diagonal matrix L_d and a strictly lower triangular matrix L_o
    This module utilizes the input "q" to predict L_d, L_o which then produce L and M    
"""
class M_NNs(nn.Module):
    def __init__(self, input_dim = 3, output_dim = 3, hidden_layer_dim = 128, num_hidden_layer = 3):

        super(M_NNs, self).__init__()
        
        self.epoch = 0

        # Define the hidden layers
        self.L_d_layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        for _ in range(num_hidden_layer - 1):
            self.L_d_layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))
        
        # Add the final output layer
        self.L_d_layers.append(nn.Linear(hidden_layer_dim, output_dim))
        
        # Define the hidden layers
        self.L_o_layers = nn.ModuleList([nn.Linear(input_dim, hidden_layer_dim)])
        for _ in range(num_hidden_layer - 1):
            self.L_o_layers.append(nn.Linear(hidden_layer_dim, hidden_layer_dim))
        
        # Add the final output layer
        self.L_o_layers.append(nn.Linear(hidden_layer_dim, output_dim))

        # Choose activation function for hidden layers
        self.activation = nn.Softplus()

        # Softplus for the last activation layer
        self.last_activation = nn.Softplus()

    def forward(self, q):
        batch_size = q.shape[0]
        
        out_1 = q
        out_2 = q
        for layer in self.L_d_layers[:-1]:
            out_1 = self.activation(layer(out_1)) if isinstance(self.activation, nn.Module) else self.activation(layer(out_1))
            
        for layer in self.L_o_layers[:-1]:
            out_2 = self.activation(layer(out_2)) if isinstance(self.activation, nn.Module) else self.activation(layer(out_2))
        
        # Apply the final layer and Softplus activation
        out_1 = self.last_activation(self.L_d_layers[-1](out_1))
        out_2 = self.last_activation(self.L_o_layers[-1](out_2))
        
        L = torch.zeros((batch_size, 3, 3), device = device)
        L [:, 0, 0] = out_1[:, 0]
        L [:, 1, 1] = out_1[:, 1]
        L [:, 2, 2] = out_1[:, 2]
        
        L [:, 1, 0] = out_2[:, 0]
        L [:, 2, 0] = out_2[:, 1]
        L [:, 2, 1] = out_2[:, 2]
        
        return L @ L.T


#---Simple NNs for Gravitational matrix according to the publication---#
"""
Module Name: G_NNs
Description:
    This module predicts the elements of G(q)
    This module utilizes inputs "cos(q)" which yields the results of G(q)

"""
class G_NNs(nn.Module):
    def __init__(self, input_dim = 3, output_dim = 3, hidden_layer_dim = 128, num_hidden_layer = 3, activation='tanh'):

        super(G_NNs, self).__init__()

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
        out = self.layers[-1](out)
        out = torch.reshape(out, (batch_size, 3, 1))
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
    def __init__(self, input_dim = 3, output_dim = 1, hidden_layer_dim = 128, num_hidden_layer = 3, activation='tanh'):

        super(B_NNs, self).__init__()

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
        out = self.layers[-1](out)
        
        B = torch.ones((batch_size, 3, 1), device = device)
        B[:, 0, 0] = out[:, 0]
        B[:, 1, 0] = out[:, 1]
        B[:, 2, 0] = out[:, 2]
        
        #----------------Define K matrix---------------------#
        K11 = (s[:, 0] * torch.cos((alpha[0])) + s[:, 1] * torch.sin((alpha[0])) + f - e) * torch.sin(q[:, 0]) - s[:, 2] * torch.cos(q[:, 0])
        K22 = (s[:, 0] * torch.cos((alpha[1])) + s[:, 1] * torch.sin((alpha[1])) + f - e) * torch.sin(q[:, 1]) - s[:, 2] * torch.cos(q[:, 1])
        K33 = (s[:, 0] * torch.cos((alpha[2])) + s[:, 1] * torch.sin((alpha[2])) + f - e) * torch.sin(q[:, 2]) - s[:, 2] * torch.cos(q[:, 2])
        
        K = torch.zeros((q.shape[0], 3, 3))                                                            
        K[:, 0, 0] = K11
        K[:, 1, 1] = K22
        K[:, 2, 2] = K33
        
        #--------------------------------Define A matrix--------------------------------#
        A = torch.zeros((q.shape[0], 3, 3))                                                             # batch_size x 3 x 3
        
        A[:, 0, 0] = s[:, 0] + e * torch.cos((alpha[0])) - f * torch.cos((alpha[0])) - l_1 * torch.cos((alpha[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 0, 1] = s[:, 0] + e * torch.cos((alpha[1])) - f * torch.cos((alpha[1])) - l_1 * torch.cos((alpha[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 0, 2] = s[:, 0] + e * torch.cos((alpha[2])) - f * torch.cos((alpha[2])) - l_1 * torch.cos((alpha[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 1, 0] = s[:, 1] + e * torch.sin((alpha[0])) - f * torch.sin((alpha[0])) - l_1 * torch.sin((alpha[0])) * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 1, 1] = s[:, 1] + e * torch.sin((alpha[1])) - f * torch.sin((alpha[1])) - l_1 * torch.sin((alpha[1])) * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 1, 2] = s[:, 1] + e * torch.sin((alpha[2])) - f * torch.sin((alpha[2])) - l_1 * torch.sin((alpha[2])) * torch.cos(q[:, 2]) # shape: batch_size

        A[:, 2, 0] = s[:, 2] - l_1 * torch.cos(q[:, 0]) # shape: batch_size
        A[:, 2, 1] = s[:, 2] - l_1 * torch.cos(q[:, 1]) # shape: batch_size
        A[:, 2, 2] = s[:, 2] - l_1 * torch.cos(q[:, 2]) # shape: batch_size
        
        return  K @ torch.linalg.inv(A) @ B



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
                


#---------------------------------Getting Data from .mat files-----------------------------#
        """
        Sửa lại hàm get_Data, đưa hết các data vào dict để dễ lấy ra lấy vô (Hashmap)
        """
def get_Data(batch_sizes):
    t = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_x.mat", "r")["data_x"][3:49998, 0]).unsqueeze(1).float()        # 1996 x 1
    _step_size = torch.ones_like(t) * 0.001        # 1996 x 1

    x = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_x.mat", "r")["data_x"][3:49998, 1]).unsqueeze(1)                # 1996 x 1
    y = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_y.mat", "r")["data_y"][3:49998, 1]).unsqueeze(1)                # 1996 x 1
    z = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_z.mat", "r")["data_z"][3:49998, 1]).unsqueeze(1)                # 1996 x 1

    s = torch.cat((x, y, z), dim=1) * 0.001                                                                       # 1996 x 3

    x_Ddot = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_x_Ddot.mat", "r")["data_x_Ddot"][3:49998, 1]).unsqueeze(1) # 1996 x 1
    y_Ddot = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_y_Ddot.mat", "r")["data_y_Ddot"][3:49998, 1]).unsqueeze(1) # 1996 x 1
    z_Ddot = torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_z_Ddot.mat", "r")["data_z_Ddot"][3:49998, 1]).unsqueeze(1) # 1996 x 1

    s_Ddot = torch.cat((x_Ddot, y_Ddot, z_Ddot), dim=1) * 0.001                                                   # 1996 x 3

    tau_1 = torch.tensor(scipy.io.loadmat("C:/Users/FPTSHOP/Desktop/Project/tau_2.mat")["tau_first_row"][0, 3:49998]).squeeze().unsqueeze(1)        # 1996 x 1
    tau_2 = torch.tensor(scipy.io.loadmat("C:/Users/FPTSHOP/Desktop/Project/tau_2.mat")["tau_second_row"][0, 3:49998]).squeeze().unsqueeze(1)        # 1996 x 1
    tau_3 = torch.tensor(scipy.io.loadmat("C:/Users/FPTSHOP/Desktop/Project/tau_2.mat")["tau_third_row"][0, 3:49998]).squeeze().unsqueeze(1)        # 1996 x 1

    tau = torch.cat((tau_1, tau_2, tau_3), dim = 1)                                                               # 1996 x 3

    theta_1_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle1.mat", "r")["angle1"][3:49998, 1])).unsqueeze(1)     # 1996 x 1
    theta_2_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle2.mat", "r")["angle2"][3:49998, 1])).unsqueeze(1)     # 1996 x 1
    theta_3_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle3.mat", "r")["angle3"][3:49998, 1])).unsqueeze(1)     # 1996 x 1

    theta_1_dot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle1_dot.mat", "r")["angle1_dot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1
    theta_2_dot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle2_dot.mat", "r")["angle2_dot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1
    theta_3_dot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle3_dot.mat", "r")["angle3_dot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1

    theta_1_Ddot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle1_Ddot.mat", "r")["angle1_Ddot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1
    theta_2_Ddot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle2_Ddot.mat", "r")["angle2_Ddot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1
    theta_3_Ddot_t = torch.deg2rad(torch.tensor(h5py.File("C:/Users/FPTSHOP/Desktop/Project/simulated_angle3_Ddot.mat", "r")["angle3_Ddot"][3:49998, 1])).unsqueeze(1)                # 1996 x 1

    u_t_data = torch.cat((theta_1_t, theta_2_t, theta_3_t, theta_1_dot_t, theta_2_dot_t, theta_3_dot_t), dim = 1)               # 1996 x 6
    u_t_1_data = torch.cat((theta_1_Ddot_t, theta_2_Ddot_t, theta_3_Ddot_t), dim = 1)               # 1996 x 6

    total_points = t.shape[0]                                                                                     # 1996
    ids_total = np.arange(total_points)                                                                           # [0, 1, 2, 3, ..., 1994, 1995]

    def random_selection(id_set, size):
        np.random.seed(8386)
        return np.random.choice(id_set, size, replace=False)

    def to_tensor(ids):
        return {
            'theta'   : u_t_data[ids, 0:3].float(),
            'theta_dot' : u_t_1_data[ids, 3:6].float(),
            's' : s[ids, :].float(),
            's_Ddot': s_Ddot[ids, :].float(),
            'tau': tau[ids, :].float(),
            'u_t_1': u_t_1_data[ids, :].float(0),
        }

    id = random_selection(ids_total, batch_sizes)
    
    X_train = to_tensor(id)

    return X_train