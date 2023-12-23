# this file is to implement different control loops for the ego car
# reference: https://carla.readthedocs.io/en/0.9.6/python_api_tutorial/

import carla
import os, sys
import math
import time

sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path,"../acc_senario"))
from utils.car_sync_mode import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data
from utils.controller import FeedForward_pid_Controller
from utils.filter import CarlaIMULowPassFilter

# neural network
network_path = os.path.join(current_path,"../neural")
sys.path.append(network_path)

client = carla.Client('carla_server', 2000)
world = client.get_world()
# world = client.load_world("Town04_Opt")
world = client.load_world("Town04_Opt")

# set for the fixed simulation step ref: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#fixed-time-step
# settings = world.get_settings()
# settings.fixed_delta_seconds = 0.01
# world.apply_settings(settings)

settings = world.get_settings()
# sychronous mode
settings.synchronous_mode = True
fixed_delta_seconds = 1/120 # 200Hz
settings.fixed_delta_seconds = fixed_delta_seconds

# physics 
settings.substepping = True
settings.max_substep_delta_time = 0.05
settings.max_substeps = 10
world.apply_settings(settings)

spawn_points = world.get_map().get_spawn_points()
spawn_point = spawn_points[90]
# get the map
town_map = world.get_map()
roads = town_map.get_topology()

# planner for the ego car
sampling_resolution = 2.0
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# set the start and end point
start_point = spawn_point
end_point = carla.Transform(carla.Location(x=500.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=33.541804, z=1.720345))


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
                "target_dist": 0.0,
                "velocity":{"x":0.0,
                                "y":0.0,
                                "z":0.0,
                                },
                "distance":0.0,
                "throttle": 0.0,

                "lead_car_speed": 0.0,
                "ego_car_speed": 0.0,

                "filtered":{
                    "acceleration":{
                        "x":0.0,
                        "y":0.0,
                        "z":0.0,
                        },
                    "gyro":{
                        "x":0.0,
                        "y":0.0,
                        "z":0.0,
                        },
                            }
                }
            }

# record the running time
# init_time = world.wait_for_tick().timestamp.platform_timestamp
init_time = 0
run_time = 0


# Spawn the lead vehicle
lead_car = mCar(client, spawn_point=spawn_point, name='leader')
world.tick() 

# time.sleep(2)
########### spawn the lead vehicle 20 meters ahead of the ego vehicle
reference_vehicle_transform = lead_car.vehicle.get_transform()

# spawn the ego car
len_of_platoon=3
ego_car=[]
route_ego=[]
for i in range(len_of_platoon):
    print(i)
    forward_vector = reference_vehicle_transform.rotation.get_forward_vector()
    spawn_distance = 10.0
    new_position = reference_vehicle_transform.location - spawn_distance * forward_vector
    # Use the same orientation as the reference vehicle
    new_rotation = reference_vehicle_transform.rotation

    # Create a new transform for the spawned vehicle
    new_transform = carla.Transform(new_position, new_rotation)
    ego_car.append(mCar(client, spawn_point=new_transform,name='ego{}'.format(i)))
    ego_car[-1].get_focus() # make spectator follow the ego car
    world.tick() 
    # get the planed route for ego
    route_ego.append(grp.trace_route(new_transform.location, end_point.location))
    reference_vehicle_transform = ego_car[i].vehicle.get_transform()

# get the planed route
start_point_2 = lead_car.spawn_point
route_leader = grp.trace_route(start_point_2.location, end_point.location)


# visualize the route for leader
visualize_waypoint(client, route_leader, sampling_resolution)
# # visualize the route
# visualize_waypoint(client, route_ego, sampling_resolution)

# local planner for the leader car
for i in range(len_of_platoon):
    ego_car[i].set_global_plan(route_ego[i])
lead_car.set_global_plan(route_leader)

# set the speed of the lead car 
lead_car.set_speed(100)

# imu filter and imu data
use_filter = "simple_low_pass"

if use_filter == "simple_low_pass":
    imu_filter = CarlaIMULowPassFilter(0.8)
    imu_data = [0 for _ in range(6)]
elif use_filter == "kalman":
    pass

# camera
def loop_5ms_loop(loop_name="5ms loop", run_time=None):
    # >>>>> send data to plotjuggler >>>>>>>>
    ego_car[0].update_state(None)

    data_to_send["custom data"]["acceleration"]["x"] = ego_car[0]._acceleration.x
    data_to_send["custom data"]["acceleration"]["y"] = ego_car[0]._acceleration.y
    data_to_send["custom data"]["acceleration"]["z"] = ego_car[0]._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car[0]._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car[0]._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car[0]._velocity.z

    # data_to_send["custom data"]["velocity"]["x"] = lead_car._velocity.x
    # data_to_send["custom data"]["velocity"]["y"] = lead_car._velocity.y
    # data_to_send["custom data"]["velocity"]["z"] = lead_car._velocity.z
    # <<<<<< send data to plotjuggler <<<<<<<<<

    # >>>>>>>>>>>>>>>>> filt >>>>>>>>>>>>>>>>>>

    # a simple low pass filter
    imu_filter.update(ego_car[0].imu_data)
    imu_data = imu_filter.state
    # <<<<<<<<<<<<<<<<< filt <<<<<<<<<<<<<<<<<<<

    # >>>>>> send data to plotjuggler >>>>>>>>
    data_to_send["custom data"]["filtered"]["acceleration"]["x"] = imu_data[0]
    data_to_send["custom data"]["filtered"]["acceleration"]["y"] = imu_data[1]
    data_to_send["custom data"]["filtered"]["acceleration"]["z"] = imu_data[2]

    data_to_send["custom data"]["filtered"]["gyro"]["x"] = imu_data[3]
    data_to_send["custom data"]["filtered"]["gyro"]["y"] = imu_data[4]
    data_to_send["custom data"]["filtered"]["gyro"]["z"] = imu_data[5]
    # <<<<<< send data to plotjuggler <<<<<<<<<

def loop_10ms_loop(loop_name="10ms loop", target_distance=10, run_time=None):


    done = lead_car.lp_control_run_step()
    # world.tick()
    target_location = lead_car.vehicle.get_transform().location
    
    for i in range(len_of_platoon):
        distance = ego_car[i]._location.distance(target_location)
        distance_error =  distance- target_distance
        throttle = controller[i].control(distance_error)
        done = ego_car[i].lp_control_run_step(throttle=throttle)
        target_location = ego_car[i].vehicle.get_transform().location

    # print(f"distance error: {distance_error}")
    data_to_send["custom data"]["throttle"] = throttle
    data_to_send["custom data"]["target_dist"] = target_distance
    data_to_send["custom data"]["distance"] = distance
    data_to_send["custom data"]["lead_car_speed"] = lead_car._velocity.x
    data_to_send["custom data"]["ego_car[0]_speed"] = ego_car[0]._velocity.x

    return done


def loop_20ms_loop(loop_name="20ms loop"):
    # this loop is for MPC
    target_dist = 20
    return target_dist
    


# variables for the loop
record_5ms = 0
record_10ms = 0
record_20ms = 0

# variables
throttle = 0
target_acc = 0
target_vel = 0
target_dist = 0

# controller
kp = 15
kd = 200
ki = 0.1
controller=[]
for i in range(len_of_platoon):
    controller.append(FeedForward_pid_Controller(kp,kd,ki))


done = False
count = 0   
while True:
    # >>>>>>>>>>> record program execution time >>>>>>>>>>>>
    record_start_time = time.time()
    # <<<<<<<<<<<< record program execution time <<<<<<<<<<<<


    # >>>>>>>>>>>>>>>>>>> get focus >>>>>>>>>>>>>>>>>>>
    run_time = run_time + fixed_delta_seconds
    world.tick()
    ego_car[-1].get_focus()
    # <<<<<<<<<<<<<<<<<<< get focus <<<<<<<<<<<<<<<<<<<


    # >>>>>>>>>>>>>>>>>> run the loop >>>>>>>>>>>>>>>>>>
    if run_time - record_5ms > 0.005:
        loop_5ms_loop(run_time=run_time)
        record_5ms = run_time
    
    if run_time - record_10ms > 0.01:
        # inner loop for speed control
        done = loop_10ms_loop(target_distance=target_dist, run_time=run_time)
        record_10ms = run_time
    
    if run_time - record_20ms > 0.02:
        # outer loop for MPC
        target_dist = loop_20ms_loop()
        record_20ms = run_time
    
    # check if local planner reach the end
    # print(f"run time: {run_time}")
    # print(f"ego car location: {ego_car[0]._location}")
    # <<<<<<<<<<<<<<<<<<<<<< run the loop <<<<<<<<<<<<<<<<<<<<<<



    # >>>>>>> update the time stamp & send the data for plot juggler >>>>>>>>>
    snapshot = world.get_snapshot()
    platform_timestamp = snapshot.timestamp.platform_timestamp
    data_to_send["timestamp"] = platform_timestamp
    send_custom_data(data_to_send)
    # <<<<<<<< update the time stamp & send the data for plot juggler <<<<<<<<<<<<


    # >>>> if running just for visualization >>>>>>>>>>>
    # make it more real to the real time
    duration = time.time() - record_start_time
    scale=1
    if duration < (fixed_delta_seconds*scale):
        time.sleep((fixed_delta_seconds *scale)- duration)
    # <<<<< if running just for visualization <<<<<<<<<<


    if run_time > 200 or done:
        # end the thread
        break

ego_car[0].destroy()

world.tick() # to make sure the client receives the last data, and the vehicle is destroyed before the client
