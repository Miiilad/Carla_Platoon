import carla
import os, sys
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import LocalPlanner
from utils.car import mCar
from utils.visualizer import visualize_waypoint


client = carla.Client('carla_server', 2000)
world = client.get_world()
world = client.load_world("Town04_Opt")

# get the spawn point
spawn_points = world.get_map().get_spawn_points()
spawn_point = spawn_points[171]

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

# acc senario
while True:
    # run the leader car
    lead_car.run_speed = 50
    done = lead_car.lp_control_run_step()

    # calculate the distance between the ego car and the leader car
    ego_tf = ego_car.vehicle.get_transform()
    leader_tf = lead_car.vehicle.get_transform()
    distance = ego_tf.location.distance(leader_tf.location)
    # print(f"distance = {distance}")

    # pid control
    target_distance = 20
    error = target_distance - distance

    kp = -10
    ego_car.run_speed = kp * error
    print(ego_car.run_speed)

    # run the ego car
    ego_car.get_focus()
    ego_car.lp_control_run_step()

    # check if local planner reach the end
    if done:
        break

# if input("press any key to continue"):
#     pass

# destroy the ego car
ego_car.destroy()