import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
model_path = os.path.join(current_path, "accel_throttle_map.pt")

import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

from model import Net

# load the model
model = Net(2, 10, 1)
model.load_state_dict(torch.load(model_path))
model.eval()

# generate random input
import pandas as pd

# the input is a dataframe with two columns: throttle and speed
# the output is a dataframe with one column: throttle

df = pd.DataFrame(columns=["throttle", "speed"])
df["throttle"] = torch.rand(100)
# the range of the speed is [0, 20]
df["speed"] = torch.rand(100) * 20

# convert the dataframe to tensor
throttle = torch.tensor(df["throttle"].values)
speed = torch.tensor(df["speed"].values)
minput = torch.stack([throttle, speed], dim=1)

import matplotlib.pyplot as plt
# predict the output, 3d plot
output = model(minput)
output = output.detach().numpy()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(df["throttle"], df["speed"], output)
ax.set_xlabel('throttle')
ax.set_ylabel('speed')
ax.set_zlabel('acceleration')
plt.show()