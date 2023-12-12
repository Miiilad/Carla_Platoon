# this file is to implement different control loops for the ego car
# reference: https://carla.readthedocs.io/en/0.9.6/python_api_tutorial/

import carla
import os, sys
import math

sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path,"../acc_senario"))
from utils.car import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data
from utils.controller import FeedForward_pid_Controller

# neural network
network_path = os.path.join(current_path,"../neural")
sys.path.append(network_path)

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
                "target_vel": 0.0,
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

# run the ego car
ego_car = mCar(client, spawn_point=spawn_point)
ego_car.set_global_plan(route)
ego_car.get_focus() # make spectator follow the ego car


def loop_5ms_loop(loop_name="5ms loop"):
    # >>>>> send data to plotjuggler >>>>>>>>
    ego_car.get_focus()
    ego_car.update_state()

    data_to_send["timestamp"] = world.wait_for_tick().frame
    data_to_send["custom data"]["acceleration"]["x"] = ego_car._acceleration.x
    data_to_send["custom data"]["acceleration"]["y"] = ego_car._acceleration.y
    data_to_send["custom data"]["acceleration"]["z"] = ego_car._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car._velocity.z

    data_to_send["custom data"]["target_acc"] = target_acc


    send_custom_data(data_to_send)
    # <<<<<< send data to plotjuggler <<<<<<<<<


def loop_10ms_loop(loop_name="10ms loop", target_acc=10):
    acc_error = target_acc - ego_car._acceleration.x
    throttle = controller.control(acc_error)
    done = ego_car.lp_control_run_step(throttle=throttle)
    data_to_send["custom data"]["throttle"] = throttle

    return done


def loop_20ms_loop(loop_name="20ms loop"):
    # this loop is for MPC
    if run_time < 5:
        target_acc = 2
    else:
        target_acc = math.sin(run_time) * 0.5
    return target_acc
    


# variables for the loop
record_5ms = 0
record_10ms = 0
record_20ms = 0

# variables
throttle = 0
target_acc = 0
target_vel = 10

# controller
controller = FeedForward_pid_Controller()

while True:
    # record the time
    snap_time = world.wait_for_tick().timestamp.platform_timestamp
    run_time = snap_time - init_time


    # run the loop
    if run_time - record_5ms > 0.005:
        loop_5ms_loop()
        record_5ms = run_time
    
    if run_time - record_10ms > 0.01:
        # inner loop for speed control
        done = loop_10ms_loop(target_acc=target_acc)
        record_10ms = run_time
    
    if run_time - record_20ms > 0.02:
        # outer loop for MPC
        target_acc = loop_20ms_loop()
        record_20ms = run_time
    
    # check if local planner reach the end
    print(f"run time: {run_time}")
    if run_time > 100 or done:
        # end the thread
        break


ego_car.destroy()