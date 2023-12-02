import pandas as pd
import os, sys

current_path = os.path.dirname(os.path.realpath(__file__))
save_path = os.path.join(current_path, "../data")
if not os.path.exists(save_path):
    os.makedirs(save_path)

df = pd.read_csv(os.path.join(save_path, "velocity_control.csv"))

import plotly

plotly.offline.init_notebook_mode(connected=True)
import plotly.graph_objs as go

# subplot
fig = plotly.tools.make_subplots(rows=2, cols=1, subplot_titles=("velocity control", "throttle"))
# plot the speed and throttle
# plot the velocity control
trace = go.Scatter(x=df["time"], y=df["speed"], mode='lines+markers',
                   name="speed")
fig.append_trace(trace, 1, 1)
# plot the throttle
trace = go.Scatter(x=df["time"], y=df["throttle"], mode='lines+markers',
                   name="throttle")
fig.append_trace(trace, 2, 1)

plotly.offline.plot(fig, filename=os.path.join(save_path, "velocity_control.html"), auto_open=True)
