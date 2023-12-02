import pandas as pd
import os, sys

current_path = os.path.dirname(os.path.realpath(__file__))
save_path = os.path.join(current_path, "../data")
if not os.path.exists(save_path):
    os.makedirs(save_path)

df = pd.read_csv(os.path.join(save_path, "accel_throttle_map.csv"))

import plotly

plotly.offline.init_notebook_mode(connected=True)
import plotly.graph_objs as go

# plot the map with different throttle
throttle = 0.2
data = []
while throttle <= 1.0:
    df_throttle = df[df["throttle"] == throttle]
    trace = go.Scatter(x=df_throttle["speed"], y=df_throttle["acceleration"], mode='lines+markers',
                       name=f"throttle: {throttle:.1f}")
    data.append(trace)
    throttle += 0.2

layout = go.Layout(
    title='Acceleration-Throttle Map',
    xaxis=dict(
        title='Speed (m/s)',
        titlefont=dict(
            family='Courier New, monospace',
            size=18,
            color='#7f7f7f'
        )
    ),
    yaxis=dict(
        title='Acceleration (m/s^2)',
        titlefont=dict(
            family='Courier New, monospace',
            size=18,
            color='#7f7f7f'
        )
    )
)
fig = go.Figure(data=data, layout=layout)
plot_path = os.path.join(save_path, 'accel_throttle_map.html')
# plot silently
plotly.offline.plot(fig, filename=plot_path)
print(f"plot saved to path: {plot_path}")
