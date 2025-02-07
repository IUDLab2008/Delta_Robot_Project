import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Subset
from sklearn.model_selection import KFold
import numpy as np

import matplotlib.pyplot as plt


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
        self.k_folds = k_folds      #Store the number of folds
        self.criterion = criterion  #Loss function
        self.dynamic_system_instance = dynamic_system_instance
        self.device = device
        
        self.train_losses = []      #Store the total Train losses over the entire process
        self.val_losses = []        #Store the total Validation losses over the entire process

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
            q = batch['theta'].float().to(self.device)
            q_dot = batch['theta_dot'].float().to(self.device)
            q_Ddot = batch['theta_Ddot'].float().to(self.device)
            s = batch['s'].float().to(self.device)
            s_dot = batch['s_dot'].float().to(self.device)
            s_Ddot = batch['s_Ddot'].float().to(self.device)
            tau = batch['tau'].float().to(self.device)
                        
            z = s[:, 2:3]
            optimizer.zero_grad()
            q_Ddot_pred = dynamic_system(q_dot, s_dot, q, z, s, s_Ddot, tau)
            loss = self.criterion(q_Ddot_pred, q_Ddot)
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

        predicted_q_Ddot_1 = []      
        actual_q_Ddot_1 = []
        
        predicted_q_Ddot_2 = []      
        actual_q_Ddot_2 = []

        predicted_q_Ddot_3 = []      
        actual_q_Ddot_3 = []

        with torch.no_grad():
            for batch in val_loader:
                q = batch['theta'].float().to(self.device)
                q_dot = batch['theta_dot'].float().to(self.device)
                q_Ddot = batch['theta_Ddot'].float().to(self.device)
                s = batch['s'].float().to(self.device)
                s_dot = batch['s_dot'].float().to(self.device)
                s_Ddot = batch['s_Ddot'].float().to(self.device)
                tau = batch['tau'].float().to(self.device)
                            
                z = s[:, 2:3]
                q_Ddot_pred = dynamic_system(q_dot, s_dot, q, z, s, s_Ddot, tau)

                predicted_q_Ddot_1.append(q_Ddot_pred[:, 0:1].cpu().numpy())
                actual_q_Ddot_1.append(q_Ddot[:, 0:1].cpu().numpy())

                predicted_q_Ddot_2.append(q_Ddot_pred[:, 1:2].cpu().numpy())
                actual_q_Ddot_2.append(q_Ddot[:, 1:2].cpu().numpy())

                predicted_q_Ddot_3.append(q_Ddot_pred[:, 2:3].cpu().numpy())
                actual_q_Ddot_3.append(q_Ddot[:, 2:3].cpu().numpy())

                loss = self.criterion(q_Ddot_pred, q_Ddot)
                val_loss += loss.item()
                
        predicted_q_Ddot_1 = np.concatenate(predicted_q_Ddot_1, axis=0)
        actual_q_Ddot_1 = np.concatenate(actual_q_Ddot_1, axis=0)

        predicted_q_Ddot_2 = np.concatenate(predicted_q_Ddot_2, axis=0)
        actual_q_Ddot_2 = np.concatenate(actual_q_Ddot_2, axis=0)

        predicted_q_Ddot_3 = np.concatenate(predicted_q_Ddot_3, axis=0)
        actual_q_Ddot_3 = np.concatenate(actual_q_Ddot_3, axis=0)
        
        fig, axes = plt.subplots(3, 1, figsize=(10, 12))

        # Plot for first motor
        axes[0].plot(actual_q_Ddot_1, label='Actual q_Ddot of Motor 1', linestyle='--', color='blue')
        axes[0].plot(predicted_q_Ddot_1, label='Predicted q_Ddot of Motor 1', color='red')
        axes[0].set_title('Motor 1: Predicted vs Actual')
        axes[0].set_xlabel('Sample')
        axes[0].set_ylabel('q_Ddot Value')
        axes[0].legend()

        # Plot for second motor
        axes[1].plot(actual_q_Ddot_2, label='Actual q_Ddot of Motor 2', linestyle='--', color='blue')
        axes[1].plot(predicted_q_Ddot_2, label='Predicted q_Ddot of Motor 2', color='red')
        axes[1].set_title('Motor 2: Predicted vs Actual')
        axes[1].set_xlabel('Sample')
        axes[1].set_ylabel('q_Ddot Value')
        axes[1].legend()

        # Plot for third motor
        axes[2].plot(actual_q_Ddot_3, label='Actual q_Ddot of Motor 3', linestyle='--', color='blue')
        axes[2].plot(predicted_q_Ddot_3, label='Predicted q_Ddot of Motor 3', color='red')
        axes[2].set_title('Motor 3: Predicted vs Actual')
        axes[2].set_xlabel('Sample')
        axes[2].set_ylabel('q_Ddot Value')
        axes[2].legend()

        # Show the plots
        plt.tight_layout()
        plt.show()

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
            
            fold_train_losses = []
            fold_val_losses = []

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

                fold_train_losses.append(train_loss)
                fold_val_losses.append(val_loss)

                print(f"Epoch {epoch + 1}/{epochs}, Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
                print(f"f_v1: {dynamic_system.f_v1.item()}, f_v2: {dynamic_system.f_v2.item()}, f_v3: {dynamic_system.f_v3.item()}")
                print(f"f_c1: {dynamic_system.f_c1.item()}, f_c2: {dynamic_system.f_c2.item()}, f_c3: {dynamic_system.f_c3.item()}")
                print('-' * 50)
            
            self.train_losses.extend(fold_train_losses)
            self.val_losses.extend(fold_val_losses)

            plt.plot(fold_train_losses, label='Train Loss')
            plt.plot(fold_val_losses, label='Validation Loss')
            plt.xlabel('Epochs')
            plt.ylabel('Loss')
            plt.title(f'Fold {fold + 1} Loss Curve')
            plt.legend()
            plt.show()

        plt.plot(self.train_losses, label='Total Train Loss')
        plt.plot(self.val_losses, label='Total Validation Loss')
        plt.xlabel('Epochs')
        plt.ylabel('Loss')
        plt.title('Total Losses Across All Folds')
        plt.legend()
        plt.show()
            
