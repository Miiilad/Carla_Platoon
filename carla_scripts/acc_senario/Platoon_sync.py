# this file is to implement different control loops for the ego car
# reference: https://carla.readthedocs.io/en/0.9.6/python_api_tutorial/

import carla
import os, sys
import math
import time,random
from utils.ResTOOL import Control,Objective
import numpy as np
import pandas as pd

from utils.attacker import DosAttacker
from utils.attacker import FDI_attacker

sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path,"../acc_senario"))
from utils.car_sync_mode import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data
from utils.controller import FeedForward_pid_Controller
from utils.filter import CarlaIMULowPassFilter
from utils.filter import KalmanFilterM
from utils.neural_learner import MyNeuralNetwork
import torch

# pandas
df = pd.DataFrame(columns=['time stamp', 'acceleration', 'yaw get_transform', 'yaw2'])

# neural network
network_path = os.path.join(current_path,"../neural")
sys.path.append(network_path)


client = carla.Client('carla_server', 2000)
world = client.get_world()
world = client.load_world("Town04_Opt")
# world = client.load_world("Town03_Opt")

# set for the fixed simulation step ref: https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#fixed-time-step
# settings = world.get_settings()
# settings.fixed_delta_seconds = 0.01
# world.apply_settings(settings)

settings = world.get_settings()
# sychronous mode
settings.synchronous_mode = True
fixed_delta_seconds = 1/200 # 200Hz
settings.fixed_delta_seconds = fixed_delta_seconds


setting={"CBF" : 0,'save_data':1, 'load_model':1, 'train_model': 1, 'save_model':1,'run_simulation': 1,  'random_spawn':0}

# attacker
from utils.attacker import configs
configs["attack_type"] = "triangle"
configs["attack_dense"] = 0.51
configs["resolution"] = 100 # the resolution of the attack profile
# attacker = DosAttacker(configs=configs)
configs["M"] = 100 # the number of signals
configs["N"] = 10
configs["E1"] = 500
configs["E2"] = 1000

configs["window_length"] = 10
attacker = FDI_attacker(configs=configs)
attack_time = np.inf # the time to start the attack

# Initialize and train the network
net = MyNeuralNetwork(path="./data/dodge/")

if setting["load_model"]:
    try: 
        net.load_model()
    except:
        print('Model not found')

try: 
     
    if setting["train_model"]:net.train_network()
    if setting["save_model"]: net.save_model()
except: 
    print('Data not found')


if not setting["run_simulation"]: sys.exit("Done")
# physics 
settings.substepping = True
settings.max_substep_delta_time = 0.05
settings.max_substeps = 10
world.apply_settings(settings)

spawn_points = world.get_map().get_spawn_points()
if setting["random_spawn"]:spawn_point = random.choice(spawn_points)
else:spawn_point = spawn_points[90]

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

# planner for the ego car
sampling_resolution = 1
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# set the start and end point
start_point = spawn_point
end_point = carla.Transform(carla.Location(x=900.665466, y=200.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=33.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=700.665466, y=100.541804, z=1.720345))

# local planner for the ego car
# ego_car.trig_autopilot()

# for plotjuggler
data_to_send = {
                "timestamp": 0,
                "custom2":{
                    "acceleration without filtered":{},
                    "yaw angle":{},
                    "yaw angle rate":{},
                    "yaw angle 2":{},
                },
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
                        },
                    "velocity":{
                        "x":0.0,
                        },
                    },
                }
            }



# record the running time
# init_time = world.wait_for_tick().timestamp.platform_timestamp
init_time = 0
run_time = 0
prev_run_time = 0


# Spawn the lead vehicle
lead_car = mCar(client, spawn_point=spawn_point, name='leader')
world.tick() 

# time.sleep(2)
########### spawn the lead vehicle 20 meters ahead of the ego vehicle
reference_vehicle_transform = lead_car.vehicle.get_transform()

# spawn the ego car
len_of_platoon=1
ego_car=[]
route_ego=[]
previous_loc_rot = []
for i in range(len_of_platoon):
    print(i)
    forward_vector = reference_vehicle_transform.rotation.get_forward_vector()
    spawn_distance = 10.0
    new_position = reference_vehicle_transform.location - spawn_distance * forward_vector
    # Use the same orientation as the reference vehicle
    new_rotation = reference_vehicle_transform.rotation

    # Create a new transform for the spawned vehicle
    new_transform = carla.Transform(new_position, new_rotation)
    previous_loc_rot.append(new_transform)
    ego_car.append(mCar(client, spawn_point=new_transform,name='ego{}'.format(i)))
    ego_car[-1].get_focus() # make spectator follow the ego car
    world.tick() 
    # get the planed route for ego
    route_ego.append(grp.trace_route(new_transform.location, end_point.location))
    reference_vehicle_transform = ego_car[i].vehicle.get_transform()

# get the planed route
start_point_2 = lead_car.spawn_point
route_leader = grp.trace_route(start_point_2.location, end_point.location)

# #Adding random traffic to the map
# # traffic_manager = client.get_trafficmanager(8000)
# blueprint_library = world.get_blueprint_library()
# vehicles = blueprint_library.filter('vehicle.*')
# spawn_points = world.get_map().get_spawn_points()
# number_of_vehicles=20
# npc_vehicles = []
# for i in range(number_of_vehicles):
#     vehicle_bp = random.choice(vehicles)
#     spawn_point = random.choice(spawn_points)
#     try: 
#         npc_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
#         npc_vehicles.append(npc_vehicle)
#     except:
#         pass
#     # npc_vehicle.set_autopilot(True)

# for vehicle in npc_vehicles:
#     vehicle.set_autopilot(True)

# visualize the route for leader
#comment if not required
#visualize_waypoint(client, route_leader, sampling_resolution)

# # visualize the route
# visualize_waypoint(client, route_ego, sampling_resolution)

# local planner for the leader car
for i in range(len_of_platoon):
    ego_car[i].set_global_plan(route_ego[i])
lead_car.set_global_plan(route_leader)

# set the speed of the lead car 
lead_car.set_speed(80)

# imu filter and imu data
use_filter = "kalman"

if use_filter == "simple_low_pass":
    imu_filter = [CarlaIMULowPassFilter(0.5) for i in range(len_of_platoon)]
    imu_data = [[0 for _ in range(2)] for i in range(len_of_platoon)]
elif use_filter == "kalman":
    # define the params of Kalman filter
    q_value = 0.1
    r_value = 0.2

    Q = np.eye(2) * q_value
    R = np.eye(2) * r_value
    P = np.eye(2) * 1000.
    x = np.array([[0], [0]])
    F = np.array([[1, 0.1], [0, 1]])
    H = np.eye(2)
    
    # def __init__(self, x_init, F, H, P, R, Q):
    imu_filter = [KalmanFilterM(x, F, H, P, R, Q) for i in range(len_of_platoon)]
    imu_data = [[0 for _ in range(2)] for i in range(len_of_platoon)] # [acceleration, velocity]
    
    
    

def update_sphere_indicator(vehicle,indicator):
    height_above_vehicle=2
    sphere_size=0.1
    vehicle_location = vehicle._location
    sphere_location = carla.Location(vehicle_location.x, vehicle_location.y, vehicle_location.z + height_above_vehicle)
    
    # Example mode determination based on speed (you can replace this with your own logic)
    if indicator:  # Example threshold for changing color
        color = carla.Color(255, 0, 0)  # Green for slow speed
    else:
        color = carla.Color(0, 255, 0)  # Red for high speed
    
    # Draw the sphere
    world.debug.draw_point(sphere_location, size=sphere_size, color=color, life_time=0.4, persistent_lines=False)

# camera
def loop_5ms_loop(previous_loc_rot,loop_name="5ms loop", run_time=None):
    # >>>>> send data to plotjuggler >>>>>>>>
    # done = lead_car.lp_control_run_step()


        # print(i,ego_car[i].imu_data.accelerometer.x,input_acceleration[i])
    # data_to_send["custom data"]["acceleration"]["y"] = ego_car[0]._acceleration.y
    # data_to_send["custom data"]["acceleration"]["z"] = ego_car[0]._acceleration.z

    data_to_send["custom data"]["velocity"]["x"] = ego_car[0]._velocity.x
    data_to_send["custom data"]["velocity"]["y"] = ego_car[0]._velocity.y
    data_to_send["custom data"]["velocity"]["z"] = ego_car[0]._velocity.z

    # data_to_send["custom data"]["velocity"]["x"] = lead_car._velocity.x
    # data_to_send["custom data"]["velocity"]["y"] = lead_car._velocity.y
    # data_to_send["custom data"]["velocity"]["z"] = lead_car._velocity.z
    # <<<<<< send data to plotjuggler <<<<<<<<<

    # >>>>>>>>>>>>>>>>> filt >>>>>>>>>>>>>>>>>>

    # a simple low pass filter
    for i in range(len_of_platoon):
        global use_filter, prev_run_time
        if use_filter == "simple_low_pass":
            imu_filter[i].update([ego_car[i].imu_data.accelerometer.x, ego_car[i]._abs_velocity])
            imu_data[i] = imu_filter[i].state
        elif use_filter == "kalman":
            imu_filter[i].kf.F[0, 1] = run_time - prev_run_time
            prev_run_time = run_time
            imu_filter[i].predict()
            imu_filter[i].update([ego_car[i].imu_data.accelerometer.x, ego_car[i]._abs_velocity])
            imu_data[i][0] = imu_filter[i].state[0]
            imu_data[i][1] = imu_filter[i].state[1]
    # <<<<<<<<<<<<<<<<< filt <<<<<<<<<<<<<<<<<<<

    # >>>>>> send data to plotjuggler >>>>>>>>
    data_to_send["custom data"]["filtered"]["acceleration"]["x"] = imu_data[0][0]
    data_to_send["custom data"]["filtered"]["velocity"]["x"] = imu_data[0][1]

    # <<<<<< send data to plotjuggler <<<<<<<<<
    [ego_car[i].update_state(None) for i in range(len_of_platoon)]
    acceleration_list=[imu_data[i][0] for i in range(len_of_platoon)]
    # acceleration_list=[ego_car[i].imu_data.accelerometer.x for i in range(len_of_platoon)]
    acceleration_lead=lead_car.imu_data.accelerometer.x

    x_k_list = []
    current_loc_rot_list = []
    for i in range(len_of_platoon):
        current_loc_rot = ego_car[i].get_transform_vec()
        current_loc_rot_list. append(current_loc_rot)
        
        slope = ego_car[i].calculate_slope(previous_loc_rot[i].location,current_loc_rot.location)
        yaw_rate = ego_car[i].calculate_yaw_rate(previous_loc_rot[i].rotation,current_loc_rot.rotation)
        print(yaw_rate)
        x_k_list.append(np.array([slope, ego_car[i].get_speed(),imu_data[i][0]])) 
        data_to_send["custom data"]["acceleration"]["{}:x".format(i)] = ego_car[i].imu_data.accelerometer.x
        data_to_send["custom2"]["acceleration without filtered"]["{}:x".format(i)] = ego_car[i].imu_data.accelerometer.x#ego_car[i]._acceleration.x
        # get the yaw angle of the vehicle
        data_to_send["custom2"]["yaw angle"]["{}:x".format(i)] = ego_car[i].vehicle.get_transform().rotation.yaw
        data_to_send["custom2"]["yaw angle rate"]["{}:x".format(i)] = ego_car[i].imu_data.gyroscope.z
        data_to_send["custom2"]["yaw angle 2"]["{}:x".format(i)] = ego_car[i].imu_data.compass

        # pandas loc append
        df.loc[len(df)] = [run_time, ego_car[i].imu_data.accelerometer.x, ego_car[i].vehicle.get_transform().rotation.yaw, ego_car[i].imu_data.compass]

    previous_loc_rot = current_loc_rot_list
    # print(x_k_list)
    
    return acceleration_list,acceleration_lead,x_k_list,previous_loc_rot


def loop_20ms_loop(loop_name="20ms loop"):
    # this loop is for MPC
    target_dist = 20
    return target_dist


def inner_control_loop(x_k_list,loop_name="10ms loop", target_distance=10):
    for i in range(len_of_platoon):
        # if input_acceleration[i]>=0:
        #     brake=0
        #     throttle=input_acceleration[i]
        # else:
        #     brake=input_acceleration[i]
        #     throttle=0
        acceleration_error =  input_acceleration[i]-acceleration_list[i]
        u_e_k = input_acceleration[i]
        x_k = x_k_list[i]
        A_d = Controller_mpc[i].Ad
        B_d = Controller_mpc[i].Bd
        # u_h_k = net.compute_u_h(x_k,u_e_k,A_d,B_d,h,h)
        # print("th , br",controller_inner[i].control_unmix(u_h_k))
        throttle,brake = controller_inner[i].control_unmix(input_acceleration[i])
        done = ego_car[i].lp_control_run_step(brake = brake, throttle=throttle)
        # target_location = ego_car[i].vehicle.get_transform().location

    # print(f"distance error: {distance_error}")
        data_to_send["custom data"]["throttle{}".format(i)] = throttle
        data_to_send["custom data"]["brake{}".format(i)] = brake
    data_to_send["custom data"]["target_dist"] = target_distance
    # data_to_send["custom data"]["distance"] = distance
    data_to_send["custom data"]["lead_car_speed"] = lead_car.get_speed()
    data_to_send["custom data"]["ego_car[0]_speed"] = ego_car[0].get_speed()

    return done

def outer_control_loop(loop_name="10ms loop", target_distance=10, run_time=None):


    # world.tick()
    location_front_vehicle = lead_car.vehicle.get_transform().location
    velocity_front_vehicle = lead_car.get_speed()
    acceleration_front_vehicle = acceleration_lead
    x_next_prediction_nom_list=[]
   
    speed_attacked_list = []
    
    x_list=[]
    for i in range(len_of_platoon):
        #Position relative
        distance = ego_car[i]._location.distance(location_front_vehicle)
        distance_error =  distance - target_distance 
        
        #Velocity relative
        # velocity_error = velocity_front_vehicle - ego_car[i].get_speed()

        speed_attacked = attacker.get_attacked_signal(ego_car[i].get_speed(), int(run_time*100))
        speed_attacked_list.append(speed_attacked)

        # start attack at the time of 5s
        if run_time < attack_time:
            velocity_error = velocity_front_vehicle - ego_car[i].get_speed()
            update_sphere_indicator(lead_car,0)
        else:
            print("Attacked!")
            velocity_error = velocity_front_vehicle - speed_attacked[0]
            update_sphere_indicator(lead_car,1)

        x = np.array([distance_error,velocity_error,acceleration_list[i]])
        x_list.append(x)
        #Calculate control
        input_acceleration[i]= Controller_mpc[i].calculate(x, acceleration_front_vehicle,u_pre_list[i], u_lim)#+3*np.sin(5*run_time+(i+1)*2)
        input_acceleration[i] += np.random.uniform(-0.5,0.5,1)[0]
        input_acceleration[i] =  np.clip(input_acceleration[i], u_lim[0], u_lim[1])

        # record u for next step: it willl be needed for control value calculation
        u_pre_list[i] = input_acceleration[i]

        #Record samples for learning

        x_next_prediction_nom_list.append(Controller_mpc[i].eval_nominal_vehicle(x_k_list[i], input_acceleration[i]))
        x_next_prediction_net=net.evaluate(x_k_list[i], input_acceleration[i])

        #Calculate the safe control through optimization
        if setting["CBF"]: input_acceleration[i]=Controller_mpc[i].Safe_Control(net,x_k_list[i],input_acceleration[i],acceleration_front_vehicle,u_lim)
        
        # print(">>>>",i, input_acceleration[i])
        location_front_vehicle = ego_car[i].vehicle.get_transform().location
        velocity_front_vehicle = ego_car[i].get_speed()
        acceleration_front_vehicle = acceleration_list[i]
        

    # print(f"distance error: {distance_error}")
    for i in range(len_of_platoon):
        data_to_send["custom data"]["acceleration"]["target{}:x".format(i)] = input_acceleration[i]
        data_to_send["custom data"]["ego_car_speed{}".format(i)] = ego_car[i].get_speed()
        data_to_send["custom data"]["distance_error{}".format(i)] = x_list[i][0]
        data_to_send["custom data"]["speed attacked{}".format(i)] = float(speed_attacked_list[i])
    data_to_send["custom data"]["target_dist"] = target_distance
    data_to_send["custom data"]["lead_car_speed"] = lead_car.get_speed()
    

    return done, input_acceleration, x_k_list, x_next_prediction_nom_list,x_next_prediction_net



# variables for the loop
record_5ms = 0
record_inner = 0
record_outer = 0
record_20ms = 0

# variables
throttle = 0
target_acc = 0
target_vel = 0
target_dist = 0

#For Longitudinal control
controller_inner=[FeedForward_pid_Controller(kp=100,ki=10,kd=60) for i in range(len_of_platoon)]

# To characterise the performance measure
R = np.diag([50])
Q = np.diag([10, 5, 0.1])
Objective = Objective(Q, R)

# Define the controller
h=0.1
prediction_H = 20
control_H = 10
u_lim= [-1,1]
Controller_mpc = [Control(h, prediction_H, control_H, Objective) for i in range(len_of_platoon)]
acceleration_list=[0]*len_of_platoon
acceleration_lead=0
input_acceleration=[0]*len_of_platoon
data_collected_input = []
data_collected_output = []
x_list_previous = []
u_pre_list=np.zeros(len_of_platoon)


done = False
count = 0   
while True:
    lead_car.lp_control_run_step()
    # >>>>>>>>>>> record program execution time >>>>>>>>>>>>
    record_start_time = time.time()
    # <<<<<<<<<<<< record program execution time <<<<<<<<<<<<


    # >>>>>>>>>>>>>>>>>>> get focus >>>>>>>>>>>>>>>>>>>
    run_time = run_time + fixed_delta_seconds
    world.tick()
    ego_car[-1].get_focus()
    # <<<<<<<<<<<<<<<<<<< get focus <<<<<<<<<<<<<<<<<<<


    # >>>>>>>>>>>>>>>>>> run the loop >>>>>>>>>>>>>>>>>>
    if run_time - record_5ms >= 0.005:
        acceleration_list,acceleration_lead,x_k_list,previous_loc_rot=loop_5ms_loop(previous_loc_rot,run_time=run_time)
        record_5ms = run_time
    
    if run_time - record_20ms >= 0.2:
        # Distance Planner
        target_dist = loop_20ms_loop()
        record_20ms = run_time


    if run_time - record_outer >= 0.1:
        # loop for speed control
        done,input_acceleration, x_list, x_next_prediction_nom_list,x_next_prediction_net = outer_control_loop(target_distance=target_dist, run_time=run_time)

        x_observed=x_list
        if len(x_list_previous) > 0:
            for i in range(len_of_platoon):
                safe_distance_for_data=7
                if x_list_previous[i][0] > (safe_distance_for_data-target_dist):
                    input = np.append(x_list_previous[i],u_implemented[i])
                    output= x_observed[i] - x_prediction[i]
                    data_collected_input.append(input)
                    data_collected_output.append(output[2])
                    # print("obsreved",x_observed[i]-x_prediction[i],x_observed[i]-x_prediction_net-x_prediction[i])
                else:
                    print(i,'th: TOO CLOSE!')

                    
        x_list_previous = x_list
        u_implemented = input_acceleration
        x_prediction = x_next_prediction_nom_list 
        x_prediction_net = x_next_prediction_net

        record_outer = run_time


    if run_time - record_inner >= 0.01:
        # inner loop for speed control
        done = inner_control_loop(x_k_list,target_distance=target_dist)
        record_inner = run_time
    



    
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
    # if duration < (fixed_delta_seconds*scale):
    #     time.sleep((fixed_delta_seconds *scale)- duration)
    # <<<<< if running just for visualization <<<<<<<<<<


    if run_time > 1000 or done:
        # Save data (optional)
        data_collected_input = np.array(data_collected_input)
        data_collected_output = np.array(data_collected_output)
        
        if setting["save_data"]:net.save_data(data_collected_input,data_collected_output)
        # torch.save(input_data, 'input_data.pt'.format())
        # torch.save(output_data, 'output_data.pt'.format())
        # end the thread
        break

for i in range(len_of_platoon):
    ego_car[i].destroy()
lead_car.destroy()

# save the data
df.to_csv('/home/docker/carla_scripts/datas.csv')

world.tick() # to make sure the client receives the last data, and the vehicle is destroyed before the client
