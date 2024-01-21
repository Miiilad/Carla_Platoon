# this file is to implement different control loops for the ego car
# reference: https://carla.readthedocs.io/en/0.9.6/python_api_tutorial/

import carla
import os, sys
import time,random
from utils.ResTOOL import Control,Objective
import numpy as np


sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(current_path,"../acc_senario"))
from utils.car_sync_mode import mCar
from utils.visualizer import visualize_waypoint
from utils.udp_server import send_custom_data
from utils.controller import FeedForward_pid_Controller
from utils.filter import CarlaIMULowPassFilter
from utils.neural_learner import MyNeuralNetwork

# neural network
network_path = os.path.join(current_path,"../neural")
sys.path.append(network_path)


client = carla.Client('carla_server', 2000)
world = client.get_world()
world = client.load_world("Town04_Opt")
# world = client.load_world("Town03_Opt")

settings = world.get_settings()
# sychronous mode
settings.synchronous_mode = True
fixed_delta_seconds = 1/200 # 200Hz
settings.fixed_delta_seconds = fixed_delta_seconds


setting={"CBF" : 0,'save_data':1, 'load_model':0, 'train_model': 0, 'save_model':0,'run_simulation': 1,  'random_spawn':1}

# Initialize and train the network
net = MyNeuralNetwork(path="./data_low/")
if setting["load_model"]:net.load_model()
if setting["train_model"]:net.train_network()
if setting["save_model"]: net.save_model()
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



def saturate(value, min=0, max=1):
    if value > max:
        value = max
    elif value < min:
        value = min
    return value

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
len_of_platoon=1
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


# local planner for the leader car
for i in range(len_of_platoon):
    ego_car[i].set_global_plan(route_ego[i])
lead_car.set_global_plan(route_leader)

# set the speed of the lead car 
lead_car.set_speed(50)
lead_car.destroy()
# imu filter and imu data
use_filter = "simple_low_pass"

if use_filter == "simple_low_pass":
    imu_filter = CarlaIMULowPassFilter(0.5)
    imu_data = [0 for _ in range(6)]
elif use_filter == "kalman":
    pass

# camera
def loop_5ms_loop(loop_name="5ms loop", run_time=None):
    # >>>>> send data to plotjuggler >>>>>>>>
    print(ego_car[0].get_speed())
    data_to_send["custom data"]["velocity"]["x"] = ego_car[0].get_speed()
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
    # <<<<<< send data to plotjuggler <<<<<<<<<
    [ego_car[i].update_state(None) for i in range(len_of_platoon)]
    acceleration_list=[ego_car[i].imu_data.accelerometer.x for i in range(len_of_platoon)]
    # acceleration_lead=lead_car.imu_data.accelerometer.x


    
    for i in range(len_of_platoon):
        data_to_send["custom data"]["acceleration"]["{}:x".format(i)] = ego_car[i].imu_data.accelerometer.x


    return acceleration_list,0

  

def inner_control_loop(th_br, target_distance=10):
    x_list=[]
    x_next_prediction_net=[]
    for i in range(len_of_platoon):
        # acceleration_error =  input_acceleration[i]-acceleration_list[i]
        throttle,brake = controller_inner[i].control_demix(th_br)
        done = ego_car[i].lp_control_run_step(brake = brake, throttle=throttle)
        # target_location = ego_car[i].vehicle.get_transform().location
                #Record samples for learning
        x = np.array([ego_car[i].get_speed(),acceleration_list[i]])
        x_list.append(x)
        # x_next_prediction_net=net.evaluate(x, th_br).reshape(2)

    # print(f"distance error: {distance_error}")
    data_to_send["custom data"]["throttle"] = throttle
    data_to_send["custom data"]["brake"] = brake
    data_to_send["custom data"]["target_dist"] = target_distance
    # data_to_send["custom data"]["distance"] = distance
    data_to_send["custom data"]["lead_car_speed"] = 0
    data_to_send["custom data"]["ego_car[0]_speed"] = ego_car[0]._velocity.x

    return done,th_br, x_list,x_next_prediction_net

def outer_control_loop(loop_name="10ms loop", target_distance=10, run_time=None):


    # world.tick()
    
    
    
    for i in range(len_of_platoon):
        # input_acceleration[i]= Controller_mpc[i].calculate(x, acceleration_front_vehicle, u_lim)
        input_acceleration[i] = np.array([2.0])

        



    # print(f"distance error: {distance_error}")
    for i in range(len_of_platoon):
        data_to_send["custom data"]["acceleration"]["target{}:x".format(i)] = input_acceleration[i].item()
    data_to_send["custom data"]["target_dist"] = target_distance
    data_to_send["custom data"]["ego_car[0]_speed"] = ego_car[0]._velocity.x

    return done, input_acceleration


def loop_20ms_loop(loop_name="20ms loop"):
    # this loop is for MPC
    target_dist = 20
    return target_dist
    


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
th_br=np.array([0.])
#For Longitudinal control
# controller=[FeedForward_pid_Controller(kp=5,ki=0,kd=50) for i in range(len_of_platoon)]
controller_inner=[FeedForward_pid_Controller(kp=100,ki=10,kd=60) for i in range(len_of_platoon)]

R = np.diag([5])
Q = np.diag([10, 1, 0])
Objective = Objective(Q, R)

# Define the controller
h=0.05
prediction_H = 20
control_H = 5
u_lim= [-70,70]
Controller_mpc = [Control(h, prediction_H, control_H, Objective) for i in range(len_of_platoon)]
acceleration_list=[0]*len_of_platoon
acceleration_lead=0
input_acceleration=[0]*len_of_platoon
data_collected_input = []
data_collected_output = []
x_list= np.zeros((1,2))
x_list_previous=[]



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
    if run_time - record_5ms >= 0.005:
        acceleration_list,accleration_lead=loop_5ms_loop(run_time=run_time)
        record_5ms = run_time



    if run_time - record_outer >= 0.05:
        # outer loop
        done,input_acceleration = outer_control_loop(target_distance=target_dist, run_time=run_time)

        record_outer = run_time



    if run_time - record_inner >= 0.01:
        # loop for acceleration control
        if  x_list[0][0]<10:
                th_br=np.array(saturate(th_br+random.uniform(-0.1,0.1),min=-1.,max=1.)+1.)
        else:
                th_br=np.array(saturate(th_br+random.uniform(-0.1,0.1),min=-1.,max=1.))       
        print(x_list) 
        th_br=np.array(saturate(th_br+random.uniform(-0.1,0.1),min=-1.,max=1.))
        done,th_br, x_list,x_next_prediction_net = inner_control_loop(th_br,target_distance=target_dist)
        x_list
        if len(x_list_previous) > 0:
            for i in range(len_of_platoon):
                input = np.append(x_list_previous[i],u_implemented)
                output= x_list[i] 
                data_collected_input.append(input)
                data_collected_output.append(output)


        x_list_previous = x_list
        u_implemented = th_br
        x_prediction_net = x_next_prediction_net
        record_inner = run_time
        

    


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


    if run_time > 100 or done:
        # Save data (optional)
        if setting["save_data"]:net.save_data(data_collected_input,data_collected_output)
        # torch.save(input_data, 'input_data.pt'.format())
        # torch.save(output_data, 'output_data.pt'.format())
        # end the thread
        break

for i in range(len_of_platoon):
    ego_car[i].destroy()


world.tick() # to make sure the client receives the last data, and the vehicle is destroyed before the client
