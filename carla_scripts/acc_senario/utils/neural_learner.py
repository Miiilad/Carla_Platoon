from utils.data_saver import DataSaver
import torch
import torch.nn as nn
import torch.optim as optim
import os
from sklearn.model_selection import train_test_split
import numpy as np

# Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer

class MyNeuralNetwork(nn.Module):
    def __init__(self,n=3,nh=10):
        # Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer
        # n = 3  # example value for n
        # nh = 10  # example value for nh
        nh1=50
        nh2=25
        super(MyNeuralNetwork, self).__init__()
        self.layer1 = nn.Linear(n + 1, nh1)
        self.layer2 = nn.Linear(nh1, nh2)
        self.output_layer = nn.Linear(nh2, n)
        self.elu = nn.ELU()

        # Initialize optimizer and loss function here, if they are fixed
        self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.parameters())

        #Data saver class instances
        #Data saver class 
        self.filename="input-output"
        self.path="./data/"
        self.DataSaver_io= DataSaver(self.path,self.filename)

    def forward(self, x, u):
        x = (x.unsqueeze(1)).reshape(1, -1) if x.dim() == 1 else x
        u = u.unsqueeze(1) if u.dim() == 1 else u
        combined = torch.cat((x, u), dim=1)
        out = self.elu(self.layer1(combined))
        out = self.elu(self.layer2(out))
        y = self.output_layer(out)
        return y

    # def load_data(self):
    #     # Replace this with actual data loading code
    #     input_data, y = torch.load(os.path.join(self.path,"input-output0"))
    #     # Assuming the last dimension should be separated
    #     x = input_data[..., :-1]  # All dimensions except the last
    #     u = input_data[..., -1:]  # Only the last dimension
    #     return x.float(), u.float(), y.float()
    
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
        epochs = 4 * len(x)
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


    def partial_derivative_u(self, x):
        """
        Evaluates the partial derivative of the network's output with respect to u at (x, 0).
        Args:
            x (Tensor): The input tensor x.

        Returns:
            Tensor: The partial derivative of the network with respect to u at (x, 0).
        """
        # Ensure x requires gradient
        x=x.astype(np.float32)
        x=torch.from_numpy(x)
        x = x.requires_grad_(True)



        u=torch.tensor([[0.0]], requires_grad=True)

        # Create a tensor for u, initialized to zero and requires gradient
        # u = torch.zeros_like(x, requires_grad=True)

        # Forward pass
        y = self.forward(x, u)

        # Initialize container for gradients
        grad_u = torch.zeros_like(y)

        # Compute gradients for each component of y
        for i in range(y.shape[1]):
            # Select the i-th component of y
            y_component = y[:, i]

            # Compute gradient with respect to u
            grad_component = torch.autograd.grad(outputs=y_component, inputs=u, 
                                                 grad_outputs=torch.ones_like(y_component), 
                                                 retain_graph=True, create_graph=True)[0]

            # Store the computed gradient
            grad_u[:, i] = grad_component.squeeze()

        return grad_u.detach().numpy().T, y.detach().numpy().T
    

    def evaluate(self,x,u):
        x=x.astype(np.float32)
        u=u.astype(np.float32)
        scalar_tensor = torch.tensor(u)
        out=self(torch.from_numpy(x),scalar_tensor.unsqueeze(0))
        

        return out.detach().numpy().T

        