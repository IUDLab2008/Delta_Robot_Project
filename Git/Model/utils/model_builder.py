import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Subset
from sklearn.model_selection import KFold
import numpy as np


class KFoldTraining:
    def __init__(self, device, dataset, k_folds, dynamic_system_instance = None, criterion = nn.MSELoss()):
        """
        Args:
            dataset (Dataset): The dataset object (e.g., MatDataLoader) to use for training and validation.
            k_folds (int): The number of folds for cross-validation.
            criterion (torch.nn.Module): The loss function to use during training.
            dynamic_system_instance: Instance of Dynamic_System Class which is composed of M, G, B, F_v, F_c Networks
        """
        self.dataset = dataset
        self.k_folds = k_folds
        self.criterion = criterion
        self.dynamic_system_instance = dynamic_system_instance
        self.device = device
        
    def __train_epoch(self, train_loader, dynamic_system, optimizer):
        """
        Perform training over number of given epochs in training dataset

        Args:
            train_loader: Training dataset
            dynamic_system: Dynamic System instance
        """
        dynamic_system.train()
        epoch_loss = 0.0
        
        for batch in train_loader:
            #---Loading data in each batch---#
            q = batch['theta'].to(self.device)
            q_dot = batch['theta_dot'].to(self.device)
            s = batch['s'].to(self.device)
            s_Ddot = batch['s_Ddot'].to(self.device)
            tau = batch['tau'].to(self.device)
            u_t_1 = batch['theta_Ddot'].to(self.device).to(torch.float32)
                        
            optimizer.zero_grad()
            q_Ddot_pred = dynamic_system(q, q_dot, s, s_Ddot, tau)
            loss = self.criterion(q_Ddot_pred, u_t_1)
            loss.backward(retain_graph=True)
            optimizer.step()
            epoch_loss += loss.item()
        return epoch_loss / len(train_loader)
    
    

    def __val_epoch(self, val_loader, dynamic_system, optimizer):
        """
        Perform validation for one epoch
        Args:
            val_loader: Validation dataset loader
            dynamic_system: Dynamic System instance
        """
        dynamic_system.eval()
        val_loss = 0.0
        
        with torch.no_grad():
            for batch in val_loader:
                # Move batch data to device
                q = batch['theta'].to(self.device)
                q_dot = batch['theta_dot'].to(self.device)
                s = batch['s'].to(self.device)
                s_Ddot = batch['s_Ddot'].to(self.device)
                tau = batch['tau'].to(self.device)
                u_t_1 = batch['theta_Ddot'].to(self.device).to(torch.float32)
                
                q_Ddot_pred = dynamic_system(q, q_dot, s, s_Ddot, tau)
                loss = self.criterion(q_Ddot_pred, u_t_1)
                val_loss += loss.item()
                
        return val_loss / len(val_loader)
    
    
        
    def train(self, epochs , batch_size , learning_rate):
        """
        Perform training over number of given epochs

        Args:
            model: Passed-in DeLan model
            optimizer: The optimizer to use
            epochs: Total number of epochs
            batch_size: Size of each batch
        """
        #---Initialize splitting into given k_folds sections---#
        kfold = KFold(n_splits=self.k_folds, shuffle=True)
                
        #---Iterating over splitted folds---#
        for fold, (train_ids, val_ids) in enumerate(kfold.split(range(len(self.dataset)))):
            
            print(f'\nFold {fold + 1}/{self.k_folds}')
            print(f'Training size: {len(train_ids)}, Validation size: {len(val_ids)}')
            print('-' * 50)
            
            #---Deduct indexes for training and validation---#
            train_subsampler = Subset(self.dataset, train_ids)
            val_subsampler = Subset(self.dataset, val_ids)
            
            #---Loading training and validation set for resulted indexes---#
            train_loader = DataLoader(train_subsampler, batch_size = batch_size, shuffle=True)
            val_loader = DataLoader(val_subsampler, batch_size = batch_size, shuffle=False)
            
            #---Define optimizer to optimize all sub-network parameters---#
            dynamic_system = self.dynamic_system_instance.to(self.device)
            optimizer = torch.optim.Adam(dynamic_system.parameters(), lr = learning_rate)
                        
            #---Loop over given epochs---#
            for epoch in range(epochs):
                
                train_loss = self.__train_epoch(train_loader, dynamic_system, optimizer)
                val_loss = self.__val_epoch(val_loader, dynamic_system, optimizer)

                print(f"Epoch {epoch + 1}/{epochs}, Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
                print(f"f_v1: {dynamic_system.f_v1.item()}, f_v2: {dynamic_system.f_v2.item()}, f_v3: {dynamic_system.f_v3.item()}")
                print(f"f_c1: {dynamic_system.f_c1.item()}, f_c2: {dynamic_system.f_c2.item()}, f_c3: {dynamic_system.f_c3.item()}")
                print('-' * 50)
            
