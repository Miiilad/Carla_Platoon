import torch
import torch.nn as nn
import torch.optim as optim
import os
from sklearn.model_selection import train_test_split
import numpy as np
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt

try:
    from utils.data_saver import DataSaver
except:
    pass
# Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer

class MyNeuralNetwork(nn.Module):
    def __init__(self,path="./data/",n=3, nh1=100, nh2=50, output_dim=1):
        # Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer
        # n = 3  # example value for n
        # nh = 10  # example value for nh
        super(MyNeuralNetwork, self).__init__()
        # Define the neural network for f(x)
        self.f_network = nn.Sequential(
            nn.Linear(n, nh1),
            nn.ReLU(),
            nn.Linear(nh1, nh2),
            nn.ReLU(),
            nn.Linear(nh2, output_dim)
        )
        
        # Define the neural network for g(x)
        self.g_network = nn.Sequential(
            nn.Linear(n, nh1),
            nn.ReLU(),
            nn.Linear(nh1, nh2),
            nn.ReLU(),
            nn.Linear(nh2, output_dim)
        )

        # Initialize optimizer and loss function here, if they are fixed
        self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.parameters())

        #Data saver class instances
        #Data saver class 
        self.filename="input-output"
        self.path=path
        self.DataSaver_io= DataSaver(self.path,self.filename)
        


    def f_tilde(self, x):
        """Evaluate f(x)."""
        return self.f_network(x)
    
    def g_tilde(self, x):
        """Evaluate g(x)."""
        return self.g_network(x)
    
    def forward(self, x, u):
        """
        Evaluate x_next = f(x) + g(x) * u.
        
        Args:
            x (torch.Tensor): Input state tensor of shape (batch_size, 3).
            u (torch.Tensor): Input control tensor of shape (batch_size, 1).
        
        Returns:
            torch.Tensor: The next state tensor of shape (batch_size, 3).
        """
        f_x = self.f_tilde(x)  # Compute f(x)
        g_x = self.g_tilde(x)  # Compute g(x)
        x_next = f_x + g_x * u  # Compute x_next
        return x_next

    
    def load_and_slice_training_data(self):
        x_data_list = []
        u_data_list = []
        output_data_list = []

        # Loop through all files in the folder
        for file in os.listdir(self.path):
            if file.startswith(self.filename) and os.path.isfile(os.path.join(self.path, file)):
                # Load the tuple of input and output data
                input_data, output_data = torch.load(os.path.join(self.path, file))

                # Assuming the last dimension should be separated
                x_data = input_data[..., :-1]  # All dimensions except the last
                u_data = input_data[..., -1:]  # Only the last dimension
                print(file + " is loaded!:", len(x_data),"samples")

                x_data_list.append(x_data)
                u_data_list.append(u_data)
                output_data_list.append(output_data)

        # Concatenate all x, u, and output data
        all_x_data = torch.cat(x_data_list, dim=0)
        all_u_data = torch.cat(u_data_list, dim=0)
        all_output_data = torch.cat(output_data_list, dim=0)
        print("Total num of samples: ", len(all_output_data))

        return all_x_data.float(), all_u_data.float(), all_output_data.float()
    
    def save_data(self, input_d,output_d):
        print(input_d.dtype,output_d.dtype)
        self.DataSaver_io.save_file(input_d,output_d)


    def save_model(self,filename="myNN"):
        """
        Save the model's state dictionary to a specified file path.
        """
        normalization_params = {
            'mean_x': self.norm_mean_x,
            'std_x': self.norm_std_x,
            'mean_u': self.norm_mean_u,
            'std_u': self.norm_std_u
        }
        file_path = self.path
        torch.save({ 'model_state_dict': self.state_dict(),
                  'normalization_params': normalization_params}, file_path+filename)
        print(f"Model saved to {file_path}")
        
        # Plot losses
        plt.plot(self.training_losses, label='Training Loss')
        plt.plot(self.validation_losses, label='Validation Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.title('Training and Validation Loss Over Time')
        plt.legend()
        plt.savefig(file_path+ 'Train_Test_Loss_results.pdf', format='pdf')

    def load_model(self,filename="myNN"):
        """
        Load the model's state dictionary from a specified file path.
        """
        file_path = self.path
        checkpoint = torch.load(file_path+filename)
        self.load_state_dict(checkpoint['model_state_dict'])
        normalization_params = checkpoint['normalization_params']

        self.norm_mean_x = normalization_params['mean_x']
        self.norm_std_x = normalization_params['std_x']
        self.norm_mean_u = normalization_params['mean_u']
        self.norm_std_u = normalization_params['std_u']
        self.eval()  # Set the model to evaluation mode
        print(f"Model loaded from {file_path}")

    def train_network(self, epochs=20,batch_size = 32):
        x, u, y_actual = self.load_and_slice_training_data()
        for i in range(50):
            print(x[i,:],u[i],y_actual[i])

        x_train, x_val, u_train, u_val, y_train, y_val = train_test_split(
            x, u, y_actual, test_size=0.2, random_state=30)
        # epochs = 3 * len(x)
        
        self.norm_mean_x = x_train.mean(dim=0)
        self.norm_std_x = x_train.std(dim=0)
        normalized_x_train = (x_train - self.norm_mean_x) / self.norm_std_x
        
        self.norm_mean_u = u_train.mean(dim=0)
        self.norm_std_u = u_train.std(dim=0)
        normalized_u_train = (u_train - self.norm_mean_u) / self.norm_std_u
        
        normalized_x_val = (x_val - self.norm_mean_x) / self.norm_std_x
        normalized_u_val = (u_val - self.norm_mean_u) / self.norm_std_u
        # print('Normalization',self.norm_std_x,self.norm_mean_x)
        
        # Create TensorDataset and DataLoader for batching
        train_dataset = TensorDataset(normalized_x_train, normalized_u_train, y_train)
        train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
        
        # Initialize arrays to store training and validation losses
        self.training_losses = []
        self.validation_losses = []
        
        for epoch in range(epochs):
            running_loss = 0.0
            
            # Training loop
            for batch_x, batch_u, batch_y in train_loader:
                self.optimizer.zero_grad()
                y_pred = self(batch_x, batch_u)
                loss = self.criterion(y_pred, batch_y)
                loss.backward()
                self.optimizer.step()
                
                # Accumulate training loss
                running_loss += loss.item()

            # Calculate average training loss for the epoch
            avg_training_loss = running_loss / len(train_loader)
            self.training_losses.append(avg_training_loss)
            
            # Validation loss
            self.eval()  # Switch to evaluation mode (if using dropout or batch norm)
            with torch.no_grad():
                y_val_pred = self(normalized_x_val, normalized_u_val)
                val_loss = self.criterion(y_val_pred, y_val).item()
            self.validation_losses.append(val_loss)
            self.train()  # Switch back to training mode

            # Logging
            if epoch % 1 == 0:
                print(f'Epoch {epoch + 1}, Training Loss: {avg_training_loss}, Validation Loss: {val_loss}')
                
        return x_train, u_train,y_train

    # Method to evaluate accuracy (MSE in this case)
    def validate_model(self, x_test, u_test, y_test):
        # Split the data into training and validation sets

        with torch.no_grad():
            y_pred = self(x_test, u_test)
            mse = self.criterion(y_pred, y_test)
            print(f'Model MSE on Test Data: {mse.item()}')
    

    def evaluate(self,x,u):
    # Ensure x and u are in float32 for consistency
        # x = x.astype(np.float32)
        # u = u.astype(np.float32)

        # Convert x and u to torch tensors
        # x = torch.tensor(x,dtype=torch.float32)
        # u = torch.tensor(u,dtype=torch.float32)

        # # Ensure norm_mean_x and norm_std_x are tensors (convert if needed)
        # norm_mean_x = torch.tensor(self.norm_mean_x)
        # norm_std_x = torch.tensor(self.norm_std_x)
        # norm_mean_u = torch.tensor(self.norm_mean_u)
        # norm_std_u = torch.tensor(self.norm_std_u)
        
        # Normalize x and u
        normalized_x = (x - self.norm_mean_x) / self.norm_std_x
        normalized_u = (u - self.norm_mean_u) / self.norm_std_u

        # Pass normalized x and u into the model
        out = self.forward(normalized_x, normalized_u.unsqueeze(0))
        

        return out.detach().numpy().T

    def compute_u_h(self, x_k, u_e_k, A_d, B_d, tau, tau_0, epsilon=1e-3):
        """
        Computes the value of {u^i_h}_k using the provided formula with a lower bound on the denominator.

        Args:
        - x_k : The state vector at step k.
        - u_e_k : The control input {u^i_e}_k.

        - A_d : Constant matrix A_d.
        - B_d : Constant matrix B_d.
        - tau (float): Constant value \tau.
        - tau_0 (float): Constant value \tau_0.
        - epsilon (float): A small positive value to avoid division by zero (default is 1e-6).

        Returns:
        - torch.Tensor: The computed value of {u^i_h}_k.
        """
        x_k_tensor = torch.tensor(x_k)
        x_k_tensor=x_k_tensor.float()
        # a_k (torch.Tensor): The auxiliary control input {a^i_k}.
        a_k = x_k [2]
        # Compute \Tilde{f}_w(x_k) and \Tilde{g}_w(x_k) using the provided neural networks
        f_w_xk = self.f_tilde(x_k_tensor)
        g_w_xk = self.g_tilde(x_k_tensor)

        # Compute the term inside the denominator: B_d + \Tilde{g}_w(x_k)
        denominator = torch.tensor(B_d[2],dtype=float) + g_w_xk

        # Add a lower bound to the denominator to avoid division by zero
        denominator = torch.clamp(denominator, min=epsilon)

        # Compute the numerator based on the given formula
        numerator = -A_d[2,:] @ x_k - f_w_xk + (1 - (tau / tau_0)) * a_k + (tau / tau_0) * u_e_k

        # Compute {u^i_h}_k by dividing the numerator by the denominator element-wise
        u_h_k = numerator / denominator

        return u_h_k
    
    
    ################################################## FOR Test ########################################
    
    
    def generate_data(self, num_samples=400):
        x = np.random.randn(num_samples, 3)
        u = np.random.randn(num_samples, 1)
        def f_true(x):
            return x[:, 2:3]
            return x[:, 0:1] + 2 * np.sin(x[:, 1:2]) + 3 * x[:, 2:3]
        def g_true(x):
            return 0
            return 1 + 0.5 * x[:, 1:2]
        y = f_true(x) + g_true(x) * u
        return torch.tensor(x, dtype=torch.float32), torch.tensor(u, dtype=torch.float32), torch.tensor(y, dtype=torch.float32)
    
    def compare(self, x, u,y_true):
        # Evaluate predictions for each sample
        y_pred_nn = np.array([self.evaluate(x_i, u_i) for x_i, u_i in zip(x, u)])
        y_pred_nn = y_pred_nn.flatten()  # Ensure y_pred_nn is a 1D array

        y_true = y_true.flatten()  # Ensure y_true is a 1D array
        
        return y_true, y_pred_nn

        
if __name__ == "__main__":
    from data_saver import DataSaver

    # Instantiate and use the network
    nn_model = MyNeuralNetwork()

    # Generate and save data
    x, u, y = nn_model.generate_data(1000)
    input = torch.cat((x, u), dim=1) 
    nn_model.save_data(input, y)

    # Train the network
    x,u,y=nn_model.train_network()

    # Evaluate and compare
    # x_test, u_test, y_true = nn_model.generate_data(100)


    y_true, y_pred_nn = nn_model.compare(x[:100,:], u[:100],y[:100])

    # Plot predictions vs. actual function
    plt.figure()
    plt.plot(y_true, label='True f(x, u)', color='blue')
    plt.plot(y_pred_nn, label='NN Prediction', linestyle='--', color='red')
    plt.legend()
    plt.title('Neural Network Predictions vs True Function')
    plt.xlabel('Sample index')
    plt.ylabel('Output')
    plt.show()







