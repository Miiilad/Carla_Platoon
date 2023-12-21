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

# spawn the ego car
ego_car = mCar(client, spawn_point=spawn_point,name='ego')
ego_car.get_focus() # make spectator follow the ego car
world.tick() 

# time.sleep(2)
########### spawn the lead vehicle 20 meters ahead of the ego vehicle
reference_vehicle_transform = ego_car.vehicle.get_transform()

forward_vector = reference_vehicle_transform.rotation.get_forward_vector()
spawn_distance = 20.0
new_position = reference_vehicle_transform.location + spawn_distance * forward_vector
# Use the same orientation as the reference vehicle
new_rotation = reference_vehicle_transform.rotation

# Create a new transform for the spawned vehicle
new_transform = carla.Transform(new_position, new_rotation)

# Spawn the lead vehicle
lead_car = mCar(client, spawn_point=new_transform, name='leader')
world.tick() 

# get the planed route
start_point_2 = lead_car.spawn_point
route2 = grp.trace_route(start_point_2.location, end_point.location)


# get the planed route
route = grp.trace_route(start_point.location, end_point.location)

# visualize the route
visualize_waypoint(client, route, sampling_resolution)
# visualize the route
visualize_waypoint(client, route, sampling_resolution)

# local planner for the leader car
ego_car.set_global_plan(route)
lead_car.set_global_plan(route2)

# set the speed of the lead car 
lead_car.set_speed(60)

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
    ego_car.update_state(None)

    data_to_send["timestamp"] = run_time
    data_to_send["custom data"]["acceleration"]["x"] = ego_car._acceleration.x
    data_to_send["custom data"]["acceleration"]["y"] = ego_car._acceleration.y
    data_to_send["custom data"]["acceleration"]["z"] = ego_car._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car._velocity.z

    data_to_send["custom data"]["velocity"]["x"] = lead_car._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = lead_car._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = lead_car._velocity.z



    send_custom_data(data_to_send)
    # <<<<<< send data to plotjuggler <<<<<<<<<

    # >>>>>>>>>>>>>>>>> filt >>>>>>>>>>>>>>>>>>

    # a simple low pass filter
    imu_filter.update(ego_car.imu_data)
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
    world.tick()
    leader_tf = lead_car.vehicle.get_transform()



    # vel_error = target_vel - ego_car._velocity.x
    distance = ego_car._location.distance(leader_tf.location)
    distance_error =  distance- target_distance
    throttle = controller.control(distance_error)
    data_to_send["custom data"]["throttle"] = throttle
    done = ego_car.lp_control_run_step(throttle=throttle)
    print(f"distance error: {distance_error}")

    data_to_send["custom data"]["target_dist"] = target_distance
    data_to_send["custom data"]["distance"] = distance
    data_to_send["custom data"]["lead_car_speed"] = lead_car._velocity.x
    data_to_send["custom data"]["ego_car_speed"] = ego_car._velocity.x

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
target_vel = 10
target_dist = 10

# controller
controller = FeedForward_pid_Controller()
controller.kp = 15
controller.kd = 200
controller.ki = 0.1

done = False

while True:
    # >>>>>>>>>>> record program execution time >>>>>>>>>>>>
    record_start_time = time.time()
    # <<<<<<<<<<<< record program execution time <<<<<<<<<<<<


    # >>>>>>>>>>>>>>>>>>> get focus >>>>>>>>>>>>>>>>>>>
    run_time = run_time + fixed_delta_seconds
    world.tick()
    ego_car.get_focus()
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
    # print(f"ego car location: {ego_car._location}")
    # <<<<<<<<<<<<<<<<<<<<<< run the loop <<<<<<<<<<<<<<<<<<<<<<


    # >>>> if running just for visualization >>>>>>>>>>>
    # make it more real to the real time
    duration = time.time() - record_start_time
    if duration < fixed_delta_seconds:
        time.sleep(fixed_delta_seconds - duration)
    # <<<<< if running just for visualization <<<<<<<<<<


    if run_time > 100 or done:
        # end the thread
        break

ego_car.destroy()

world.tick() # to make sure the client receives the last data, and the vehicle is destroyed before the client
