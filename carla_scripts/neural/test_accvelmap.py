import os, sys
current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
model_path = os.path.join(current_path, "accel_throttle_map.pt")

import torch
from model import Net

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(f"device: {device}")

# load the model
model = Net(2, 10, 1)
model.load_state_dict(torch.load(model_path))
model.eval()

# generate random input
import pandas as pd

# the input is a dataframe with two columns: throttle and speed
# the output is a dataframe with one column: throttle

df = pd.DataFrame(columns=["acceleration", "speed"])
df["acceleration"] = torch.rand(1000) * 20
# the range of the speed is [0, 20]
df["speed"] = torch.rand(1000) * 20

# convert the dataframe to tensor
acceleration = torch.tensor(df["acceleration"].values)
speed = torch.tensor(df["speed"].values)
minput = torch.stack([acceleration, speed], dim=1)

# test 1 example
# ["", "speed"]
tempinput = torch.tensor([5.0, 0])
output = model(tempinput)
print(f"the throttle is: {output.detach().numpy()[0]} m/s^2")

import matplotlib.pyplot as plt

# Assuming 'output', 'df["acceleration"]', and 'df["speed"]' are defined and valid
output = model(minput)
output = output.detach().numpy()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Use the output values for color mapping
sc = ax.scatter(df["acceleration"], df["speed"], output, c=output, cmap=plt.cm.jet)

# Set labels
ax.set_xlabel('acceleration')
ax.set_ylabel('speed')
ax.set_zlabel('throttle')

# Create a colorbar
cbar = plt.colorbar(sc, ax=ax)
cbar.set_label('throttle')

plt.show()

# # Assuming 'output', 'df["acceleration"]', and 'df["speed"]' are defined and valid
# output = model(minput)
# output = output.detach().numpy()

# fig = plt.figure(figsize=(18, 6))

# # First view
# ax1 = fig.add_subplot(131, projection='3d')
# sc1 = ax1.scatter(df["acceleration"], df["speed"], output, c=output, cmap=plt.cm.jet)
# ax1.view_init(30, 45)  # Elevation and azimuth angle
# ax1.set_xlabel('acceleration')
# ax1.set_ylabel('speed')
# ax1.set_zlabel('acceleration')

# # Second view
# ax2 = fig.add_subplot(132, projection='3d')
# sc2 = ax2.scatter(df["acceleration"], df["speed"], output, c=output, cmap=plt.cm.jet)
# ax2.view_init(30, 135)  # Elevation and azimuth angle
# ax2.set_xlabel('acceleration')
# ax2.set_ylabel('speed')
# ax2.set_zlabel('acceleration')

# # Third view
# ax3 = fig.add_subplot(133, projection='3d')
# sc3 = ax3.scatter(df["acceleration"], df["speed"], output, c=output, cmap=plt.cm.jet)
# ax3.view_init(30, 225)  # Elevation and azimuth angle
# ax3.set_xlabel('acceleration')
# ax3.set_ylabel('speed')
# ax3.set_zlabel('acceleration')

# # Create a colorbar
# cbar = plt.colorbar(sc3, ax=ax3, pad=0.1)
# cbar.set_label('acceleration')

# plt.tight_layout()
# plt.show()


