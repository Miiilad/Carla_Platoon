from utils.data_saver import DataSaver
import torch
import torch.nn as nn
import torch.optim as optim
import os


# Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer
n = 5  # example value for n
nh = 10  # example value for nh

class MyNeuralNetwork(nn.Module):
    def __init__(self,n=3,nh=10):
        # Assuming n is the dimension of x and y, and nh is the number of neurons in each hidden layer
        # n = 3  # example value for n
        # nh = 10  # example value for nh
        super(MyNeuralNetwork, self).__init__()
        self.layer1 = nn.Linear(n + 1, nh)
        self.layer2 = nn.Linear(nh, nh)
        self.output_layer = nn.Linear(nh, n)
        self.elu = nn.ELU()

        # Initialize optimizer and loss function here, if they are fixed
        self.criterion = nn.MSELoss()
        self.optimizer = optim.Adam(self.parameters())

        #Data saver class instances
        #Data saver class 
        self.DataSaver_io= DataSaver("./data/","input-output")

    def forward(self, x, u):
        combined = torch.cat((x, u), dim=1)
        out = self.elu(self.layer1(combined))
        out = self.elu(self.layer2(out))
        y = self.output_layer(out)
        return y

    def load_data(self):
        # Replace this with actual data loading code
        input_data, y = torch.load(os.path.join("./data/","input-output0"))
        # Assuming the last dimension should be separated
        x = input_data[..., :-1]  # All dimensions except the last
        u = input_data[..., -1:]  # Only the last dimension
        return x.float(), u.float(), y.float()
    
    def save_data(self, input_d,output_d):
        self.DataSaver_io.save_file(input_d,output_d)

    def train_network(self, epochs=1000):
        x, u, y_actual = self.load_data()
        for epoch in range(epochs):
            self.optimizer.zero_grad()
            y_pred = self(x, u)
            loss = self.criterion(y_pred, y_actual)
            loss.backward()
            self.optimizer.step()

            if epoch % 100 == 99:
                print(f'Epoch {epoch + 1}, Loss: {loss.item()}')