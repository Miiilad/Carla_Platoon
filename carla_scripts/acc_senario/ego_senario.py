import carla
import os, sys
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

from utils.car import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data


client = carla.Client('carla_server', 2000)
world = client.get_world()
# world = client.load_world("Town04_Opt")
world = client.load_world("Town04_Opt")

# spawn ego car
ego_car = mCar(client)
ego_car.get_focus() # make spectator follow the ego car

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

# planner for the ego car
sampling_resolution = 2.0
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# set the start and end point
start_point = ego_car.spawn_point
end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=33.541804, z=1.720345))

# get the planed route
route = grp.trace_route(start_point.location, end_point.location)

# visualize the route
visualize_waypoint(client, route, sampling_resolution)

# local planner for the ego car
ego_car.set_global_plan(route)
ego_car.trig_autopilot()

# for plotjuggler
data_to_send = {
                "custom data":{
                "timestamp": 0,
                "acceleration":{"x":0.0,
                                "y":0.0,
                                "z":0.0,
                                },
                "velocity":{"x":0.0,
                                "y":0.0,
                                "z":0.0,
                                },}
            }

# run the ego car
while True:
    ego_car.run_speed = 50
    
    # done = ego_car.lp_control_run_step()
    ego_car.get_focus()
    ego_car.update_state()

    data_to_send["custom data"]["timestamp"] = world.wait_for_tick().frame
    data_to_send["custom data"]["acceleration"]["x"] = ego_car._acceleration.x
    data_to_send["custom data"]["acceleration"]["y"] = ego_car._acceleration.y
    data_to_send["custom data"]["acceleration"]["z"] = ego_car._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car._velocity.z

    send_custom_data(data_to_send)
    # check if local planner reach the end
    # if done:
    #     break

# destroy the ego car
ego_car.destroy()