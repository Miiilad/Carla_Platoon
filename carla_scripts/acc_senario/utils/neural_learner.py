from utils.data_saver import DataSaver
import torch
import torch.nn as nn
import torch.optim as optim
import os
from sklearn.model_selection import train_test_split
import numpy as np

# Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer

class MyNeuralNetwork(nn.Module):
    def __init__(self,path="./data/",n=3, nh1=20, nh2=20, output_dim=1):
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
        self.DataSaver_io.save_file(input_d,output_d)

    def save_model(self,filename="myNN"):
        """
        Save the model's state dictionary to a specified file path.
        """
        file_path = self.path
        torch.save(self.state_dict(), file_path+filename)
        print(f"Model saved to {file_path}")

    def load_model(self,filename="myNN"):
        """
        Load the model's state dictionary from a specified file path.
        """
        file_path = self.path
        self.load_state_dict(torch.load(file_path+filename))
        self.eval()  # Set the model to evaluation mode
        print(f"Model loaded from {file_path}")

    def train_network(self, epochs=1000):
        x, u, y_actual = self.load_and_slice_training_data()
        x_train, x_val, u_train, u_val, y_train, y_val = train_test_split(
            x, u, y_actual, test_size=0.2, random_state=30)
        epochs = 3 * len(x)
        for epoch in range(epochs):
            self.optimizer.zero_grad()
            y_pred = self(x_train, u_train)
            loss = self.criterion(y_pred, y_train)
            loss.backward()
            self.optimizer.step()

            if epoch % 100 == 99:
                print(f'Epoch {epoch + 1}, Loss: {loss.item()}')
                self.validate_model(x_val,u_val,y_val)

    
    # Method to evaluate accuracy (MSE in this case)
    def validate_model(self, x_test, u_test, y_test):
        # Split the data into training and validation sets

        with torch.no_grad():
            y_pred = self(x_test, u_test)
            mse = self.criterion(y_pred, y_test)
            print(f'Model MSE on Test Data: {mse.item()}')
    

    def evaluate(self,x,u):
        x=x.astype(np.float32)
        u=u.astype(np.float32)
        scalar_tensor = torch.tensor(u)
        out=self(torch.from_numpy(x),scalar_tensor.unsqueeze(0))
        

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
        



# Function to generate sample data
def generate_sample_data(num_samples=1000):
    # Randomly generate x, u, and the corresponding y = f(x) + g(x) * u
    x = np.random.randn(num_samples, 3).astype(np.float32)  # x has 3 dimensions
    u = np.random.randn(num_samples, 1).astype(np.float32)  # u is 1-dimensional
    
    # Here we're creating y as a simple function for demonstration, ideally this would be modeled data
    y = (np.sum(x, axis=1, keepdims=True) + u).astype(np.float32)  # Dummy function for y = f(x) + g(x) * u
    
    # Convert x and u to PyTorch tensors
    x_tensor = torch.tensor(x)
    u_tensor = torch.tensor(u)
    y_tensor = torch.tensor(y)
    
    # Combine x and u for saving as input
    input_data = torch.cat((x_tensor, u_tensor), dim=1)
    return input_data, y_tensor

# Define a function to test the MyNeuralNetwork class
def test_my_neural_network():
    # Create the model instance
    model = MyNeuralNetwork()

    # Generate sample data
    input_data, output_data = generate_sample_data(num_samples=1000)
    
    # Save the data using the save_data method
    model.save_data(input_data, output_data)
    print("Sample data saved.")

    # Load the data
    x, u, y = model.load_and_slice_training_data()
    print("Data loaded:", x.shape, u.shape, y.shape)

    # Train the model
    print("Training the model...")
    model.train_network(epochs=100)
    print("Training complete.")

    # Evaluate on a new sample
    sample_x = np.random.randn(3).astype(np.float32)  # A random sample of x
    sample_u = np.random.randn(1).astype(np.float32)  # A random sample of u
    result = model.evaluate(sample_x, sample_u)
    print(f"Evaluation result: {result}")




# Run the test function
# test_my_neural_network()