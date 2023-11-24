import carla

client = carla.Client('carla_server', 2000)

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()

# create vehicle
vehicle_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
start_point = spawn_points[0]
vehicle = world.spawn_actor(vehicle_bp, start_point)

# get the car's position on the map
vehicle_pos = vehicle.get_transform()
print(vehicle_pos)

# initial spawn point is the same = just 0.6m higher off the ground
print(start_point)

# autopilot
vehicle.set_autopilot(True)

# get actual position from the car moving
vechicle_pos = vehicle.get_transform()
print(vehicle_pos)

# get the map
town_map = world.get_map()
roads = town_map.get_topology()

import sys
sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.global_route_planner import GlobalRoutePlanner

sampling_resolution = 2.0

grp = GlobalRoutePlanner(town_map, sampling_resolution)

point_a = carla.Location(x = 64.644844, y = 24.471010, z = 0.6000)
point_b = carla.Location(x = -114.478943, y = 65.782814, z = -0.003669)

route = grp.trace_route(point_a, point_b)

for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, 'O', draw_shadow=False,
                            color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                            persistent_lines=True)
    world.debug.draw_point(waypoint[0].transform.location, size=0.2,
                           color=carla.Color(r=0, g=255, b=0), life_time=120.0,
                           persistent_lines=True)

for actor in world.get_actors().filter('*vehicle.*'):
    actor.destroy()

for sensor in world.get_actors().filter('*sensor.*'):
    sensor.destroy()

truck_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
mini_bp = world.get_blueprint_library().find('vehicle.tesla.model3')

truck = world.try_spawn_actor(truck_bp, start_point)

spectator = world.get_spectator()
spawn_points = world.get_map().get_spawn_points()
start_point = spawn_points[0]

spectator_pos = carla.Transform(start_point.location + carla.Location(z=50),
                                carla.Rotation(pitch=-90))

spectator.set_transform(spectator_pos)