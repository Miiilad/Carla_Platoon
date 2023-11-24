import carla
import random
import time

def main():
    #create client，and get world
    client = carla.Client("carla_server", 2000)
    world = client.get_world()
    # world = client.load_world("Town10HD_Opt")
    print("available maps: ", client.get_available_maps())
    world = client.load_world("Town04_Opt")

    spawn_points = world.get_map().get_spawn_points()
    
    # visualize the spawning point
    for i, spawn_point in enumerate(spawn_points):
        world.debug.draw_string(spawn_point.location, str(i), life_time=100)
        world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

    
    spectator_tf = carla.Transform()
    # spectator_tf.location = carla.Location(x=-75.759834, y=39.924366, z=262.473206)
    spectator_tf.location = carla.Location(x=226.5635, y=30.00, z=6.463309)
    # spectator_tf.rotation = carla.Rotation(pitch=-79.805153, yaw=0.00, roll=0.00)
    spectator = world.get_spectator()
    spectator.set_transform(spectator_tf)

    while True:
        spectator_tf = spectator.get_transform()
        print(spectator_tf)

if __name__ == "__main__":
    main()