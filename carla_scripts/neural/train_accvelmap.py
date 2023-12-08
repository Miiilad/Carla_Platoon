import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

# data processing
from sklearn.model_selection import train_test_split

# visualize
import tqdm
from torch.utils.tensorboard import SummaryWriter

# specify the path to the data, and the model
import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(current_path, "..","data","accel_throttle_map.csv")
model_save_path = os.path.join(current_path,"accel_throttle_map.pt")
log_path = os.path.join(current_path, "log")
if not os.path.exists(log_path):
    os.makedirs(log_path)

import pandas as pd

# specify the device and the model
from model import Net
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"device: {device}")

# data
if not os.path.exists(data_path):
    print(f"no data found at {data_path}, please run carla_scripts/control/accel_throttle_map.py first")
    sys.exit(0)
data = pd.read_csv(data_path)

# filt the data
# Apply the first condition
condition1 = (data['acceleration'] < 20) & (data['speed'].abs() < 0.1)
# Apply the second condition
condition2 = data['speed'] > 30
# Combine conditions with OR (|) since rows should be deleted if they meet either condition
combined_condition = condition1 | condition2
# Invert the combined condition and use it to filter the DataFrame
data = data[~combined_condition]


# self defined model
train_size = 0.99
learning_rate = 5e-4
batch_size = 32
epochs = 1000

class CustomDataset(Dataset):
    def __init__(self, n_input, n_output):
        self.input = n_input
        self.output = n_output

    def __len__(self):
        return len(self.input)
    
    def __getitem__(self, idx):
        return self.input[idx], self.output[idx]
    
def main():
    train_x, test_x, train_y, test_y = train_test_split(data[["throttle", "speed"]].values, data[["acceleration"]].values, test_size=1-train_size)
    train_dataset = CustomDataset(train_x, train_y)
    test_dataset = CustomDataset(test_x, test_y)

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=True)

    model = Net(2, 10, 1).to(device)
    model.train()

    # loss and optimizer
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    # tensorboard
    writer = SummaryWriter(log_dir=log_path)

    # train
    for epoch in range(epochs):
        pbar = tqdm.tqdm(train_loader)
        for step, (x, y) in enumerate(pbar):
            x = x.to(device).float() # ensure the data type is float
            y = y.to(device).float() # ensure the data type is float

            # forward
            y_pred = model(x)
            loss = criterion(y_pred, y)

            # backward
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            # visualize
            writer.add_scalar("loss", loss, epoch*len(train_loader)+step)
            pbar.set_description(f"loss: {loss.item():.4f}")
    
    # save the model
    torch.save(model.state_dict(), model_save_path)
    
if __name__ == "__main__":
    main()