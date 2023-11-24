import carla

def visualize_waypoint(client, waypoints, resolution):
    world = client.get_world()
    location_temp = None
    for waypoint in waypoints:
        # get forward vector
        if location_temp is not None:
            vector = (waypoint[0].transform.location - location_temp)/resolution
            world.debug.draw_arrow(waypoint[0].transform.location + carla.Location(z=0.5), waypoint[0].transform.location + carla.Location(z=0.5) + vector, life_time=100)
        location_temp = waypoint[0].transform.location