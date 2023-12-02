# reference: https://carla.readthedocs.io/en/0.9.6/python_api_tutorial/

import carla
import os, sys
import pandas as pd
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

from utils.car import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data


client = carla.Client('carla_server', 2000)
world = client.get_world()
# world = client.load_world("Town04_Opt")
world = client.load_world("Town04_Opt")

spawn_points = world.get_map().get_spawn_points()
spawn_point = spawn_points[171]

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

# planner for the ego car
sampling_resolution = 2.0
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# set the start and end point
start_point = spawn_point
end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=33.541804, z=1.720345))

# get the planed route
route = grp.trace_route(start_point.location, end_point.location)

# visualize the route
visualize_waypoint(client, route, sampling_resolution)

# local planner for the ego car
# ego_car.trig_autopilot()

# for plotjuggler
data_to_send = {
                "timestamp": 0,
                "custom data":{
                "acceleration":{"x":0.0,
                                "y":0.0,
                                "z":0.0,
                                },
                "velocity":{"x":0.0,
                                "y":0.0,
                                "z":0.0,
                                },
                "throttle": 0.0,
                }
            }

# record the running time
init_time = world.wait_for_tick().timestamp.platform_timestamp
run_time = 0

# create a acceleration-throttle speed map
df = pd.DataFrame(columns=["throttle", "acceleration", "speed"])

throttle = 0.2
# run the ego car
ego_car = mCar(client, spawn_point=spawn_point)
ego_car.set_global_plan(route)
ego_car.get_focus() # make spectator follow the ego car

while throttle <= 1.0:
    # record the time
    snap_time = world.wait_for_tick().timestamp.platform_timestamp
    run_time = snap_time - init_time

    done = ego_car.lp_control_run_step(throttle=throttle)
    ego_car.get_focus()
    ego_car.update_state()

    data_to_send["timestamp"] = world.wait_for_tick().frame
    data_to_send["custom data"]["acceleration"]["x"] = ego_car._acceleration.x
    data_to_send["custom data"]["acceleration"]["y"] = ego_car._acceleration.y
    data_to_send["custom data"]["acceleration"]["z"] = ego_car._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car._velocity.z

    data_to_send["custom data"]["throttle"] = throttle

    send_custom_data(data_to_send)
    # check if local planner reach the end
    # pandas concat
    new_df = pd.DataFrame({"throttle": [throttle],
                           "acceleration": [ego_car._acceleration.x],
                           "speed": [ego_car._velocity.x]})
    df = pd.concat([df, new_df], ignore_index=True)

    if done or run_time > 20:
        print(f"total running time: {run_time:.2f}, throttle: {throttle:.1f}")
        init_time = world.wait_for_tick().timestamp.platform_timestamp
        throttle += 0.2
        # destroy the ego car
        ego_car.destroy()
        ego_car = mCar(client, spawn_point=spawn_point)
        ego_car.set_global_plan(route)


ego_car.destroy()


# save the map
import os, sys

current_path = os.path.dirname(os.path.realpath(__file__))
save_path = os.path.join(current_path, "../data")
if not os.path.exists(save_path):
    os.makedirs(save_path)

# save the data
df.to_csv(os.path.join(save_path, "accel_throttle_map.csv"), index=False)