import carla
import os, sys
import random
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import LocalPlanner
from utils.car import mCar
from utils.attacker import attacker
from utils.controller import pid_controller
from utils.visualizer import visualize_waypoint

# for plot juggler
from utils.udp_server import send_custom_data

IMPLEMENT_ACTUATOR_ATTACK = False
IMPLEMENT_SENSOR_ATTACK = False
ATTACK_TIME = 5.0

client = carla.Client('carla_server', 2000)
world = client.get_world()
world = client.load_world("Town04_Opt")

# set the weather sunny and clear
weather = carla.WeatherParameters(cloudiness=0.0, precipitation=0.0, sun_altitude_angle=90.0)

# get the spawn point
spawn_points = world.get_map().get_spawn_points()
spawn_point = spawn_points[71]

# spawn ego car
ego_car = mCar(client, spawn_point=spawn_point, name='ego')
ego_car.get_focus() # make spectator follow the ego car

# spawn the lead vehicle 20 meters ahead of the ego vehicle
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

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

# planner for the ego car
sampling_resolution = 2.0
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# set the start and end point
start_point_1 = ego_car.spawn_point
start_point_2 = lead_car.spawn_point
end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))
# end_point = carla.Transform(carla.Location(x=340.665466, y=33.541804, z=1.720345))

# get the planed route
route1 = grp.trace_route(start_point_1.location, end_point.location)
route2 = grp.trace_route(start_point_2.location, end_point.location)

# visualize the route
visualize_waypoint(client, route1, sampling_resolution)

# local planner for the ego car and the leader car
ego_car.set_global_plan(route1)
lead_car.set_global_plan(route2)

# run the simulation
init_time = world.wait_for_tick().timestamp.platform_timestamp
run_time = 0

# attacker
attack_dense = 0.8
attacker = attacker(attack_dense=attack_dense, attack_time= ATTACK_TIME)

# for plot juggler
data_to_send = {
    "timestamp": 0,

    "signals":{
        "acceleration":{"x":0.0,
                        "y":0.0,
                        "z":0.0,
                        },

        "velocity":{"x":0.0,
                    "y":0.0,
                    "z":0.0,
                    },

        "control signal":0.0,
        "distance":0.0,
    }
}

# dos attack profile
collide_time_record = None
distance = None
# run time should within 100 seconds
while run_time < 100:
    snap_time = world.wait_for_tick().timestamp.platform_timestamp
    run_time = snap_time - init_time
    # print(f"time:{run_time:.2f}\tcollision: ", ego_car.collision_data) # show the time in seconds
    

    # run the leader car
    lead_car.run_speed = 40
    # if run_time > 10:
    #     lead_car.run_speed = 110
    
    done = lead_car.lp_control_run_step()

    # calculate the distance between the ego car and the leader car
    ego_tf = ego_car.vehicle.get_transform()
    leader_tf = lead_car.vehicle.get_transform()

    # this is about to implement sensor attack in the control loop
    sensor_value = ego_tf.location.distance(leader_tf.location)
    distance = attacker.dos_attack(sensor_value, run_time) if IMPLEMENT_SENSOR_ATTACK else sensor_value

    print(f"detected distance = {distance}")

    # pid control
    distance_setpoint = 10
    error = distance_setpoint - distance

    # actuator attack
    control_value = pid_controller(error)
    control_input = attacker.dos_attack(control_value, run_time) if IMPLEMENT_ACTUATOR_ATTACK else control_value
    print(f"control input: {control_input}")

    # for plot juggler
    data_to_send["timestamp"] = world.wait_for_tick().frame
    data_to_send["signals"]["acceleration"]["x"] = ego_car._acceleration.x
    data_to_send["signals"]["acceleration"]["y"] = ego_car._acceleration.y
    data_to_send["signals"]["acceleration"]["z"] = ego_car._acceleration.z

    data_to_send["signals"]["velocity"]["x"] = ego_car._velocity.x
    data_to_send["signals"]["velocity"]["y"] = ego_car._velocity.y
    data_to_send["signals"]["velocity"]["z"] = ego_car._velocity.z

    data_to_send["signals"]["control signal"] = control_input
    data_to_send["signals"]["distance"] = distance-5

    send_custom_data(data_to_send)
    
    ego_car.run_speed = control_input
    # ego_car.run_speed = 120

    # run the ego car
    ego_car.get_focus()
    ego_car.lp_control_run_step()

    # check if local planner reach the end
    if done:
        print("done")
        break
    
    if ego_car.collision_data is not None:
        # CollisionEvent(frame=5673, timestamp=21.158847, other_actor=0x447cfe0)
        collide_time = ego_car.collision_data.timestamp
        if collide_time_record is None:
            collide_time_record = run_time
        print(f"collision occured at time: {collide_time_record:.2f}")

    if (collide_time_record is not None) and (run_time - collide_time_record > 5):
        break

# if input("press any key to continue"):
#     pass

# destroy the ego car
ego_car.destroy()
lead_car.destroy()