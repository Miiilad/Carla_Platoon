import carla
import os, sys
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner
from utils.ego_car import mCar


client = carla.Client('carla_server', 2000)
world = client.get_world()
world = client.load_world("Town04_Opt")

# spawn ego car
ego_car = mCar(client)
ego_car.get_focus() # make spectator follow the ego car

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

sampling_resolution = 2.0
grp = GlobalRoutePlanner(town_map, sampling_resolution)

start_point = ego_car.spawn_point
end_point = carla.Transform(carla.Location(x=340.665466, y=37.541804, z=1.720345))

route = grp.trace_route(start_point.location, end_point.location)

# visualize the route
location_temp = None
for waypoint in route:
    # get forward vector
    if location_temp is not None:
        vector = waypoint[0].transform.location - location_temp
        world.debug.draw_arrow(waypoint[0].transform.location + carla.Location(z=0.5), waypoint[0].transform.location + vector*0.3 + carla.Location(z=0.5), life_time=100)
    location_temp = waypoint[0].transform.location
    # world.debug.draw_arrow(waypoint[0].transform.location, waypoint[0].transform.location + carla.Location(z=1), life_time=100)