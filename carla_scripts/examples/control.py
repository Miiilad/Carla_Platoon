import carla
import random

#the mode of spawning，"spectator" OR "random"
SPAWN_POINT = "random"

#the mode of control，"set_transform" OR "ackermann_control" OR "control" OR "autopilot"
CONTROL_MODE = "set_transform"

# to make the vehicle follow the line please look at https://carla.readthedocs.io/en/latest/core_map/

def main():
    #create client，and get world
    client = carla.Client("carla_server", 2000)
    world = client.get_world()
    world = client.load_world("Town04_Opt")

    #set ego car type
    ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    ego_bp.set_attribute('role_name','ego')

    if SPAWN_POINT == "spectator":
        #choose current spectator position as ego vehicle spawn point
        spectator = world.get_spectator()
        spectator_tf = spectator.get_transform()
        spawn_point = spectator_tf
    elif SPAWN_POINT == "random":
        #get random spawn point
        spawn_points = world.get_map().get_spawn_points()
        
        # visualize the spawning point
        for i, spawn_point in enumerate(spawn_points):
            world.debug.draw_string(spawn_point.location, str(i), life_time=100)
            world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

        # spawn_point = random.choice(spawn_points)
        spawn_point = spawn_points[171]
    
    #generate ego car
    ego_vehicle = world.try_spawn_actor(ego_bp, spawn_point)
    snap = world.wait_for_tick()
    init_frame = snap.frame
    run_frame = 0
    print(f"spawn ego vehicle at: {spawn_point}")

    # set the car physical performance
    bbx = ego_vehicle.bounding_box
    physics = ego_vehicle.get_physics_control()
    print(f"bounding_box = {bbx}")
    print(f"physics = {physics}")

    physics.mass = 2000
    ego_vehicle.apply_physics_control(physics)
    ego_vehicle.set_light_state(carla.VehicleLightState.All)

    # async
    while run_frame < 10000:
        snap = world.wait_for_tick()
        run_frame = snap.frame - init_frame
        print(f"-- run_frame = {run_frame}")

        # make spectator follow the ego car
        spectator = world.get_spectator()
        ego_tf = ego_vehicle.get_transform()
        spectator_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
        spectator_tf.location += ego_tf.get_forward_vector() * (-10)
        spectator_tf.location += ego_tf.get_up_vector() * 3
        spectator.set_transform(spectator_tf)

        # control
        if CONTROL_MODE == "set_transform":
            new_ego_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
            new_ego_tf.location += ego_tf.get_forward_vector() * 0.2
            new_ego_tf.location.y = 30
            ego_vehicle.set_transform(new_ego_tf)
        elif CONTROL_MODE == "ackermann_control":
            #ackermann_control
            ackermann_control = carla.VehicleAckermannControl()
            ackermann_control.speed = 5.0
            ego_vehicle.apply_ackermann_control(ackermann_control)
        elif CONTROL_MODE == "control":
            #control
            control = carla.VehicleControl()
            control.throttle = 0.3
            ego_vehicle.apply_control(control)
        else:
            #set autopilot
            ego_vehicle.set_autopilot(True)
        
        #get the status of the car
        transform = ego_vehicle.get_transform()
        velocity = ego_vehicle.get_velocity()
        acceleration = ego_vehicle.get_acceleration()
        print(f"-current transform = {transform}")
        print(f"-current velocity = {velocity}")
        print(f"-current acceleration = {acceleration}")

    # distory the car
    is_destroyed = ego_vehicle.destroy()
    if is_destroyed:
        print(f"ego_vehicle has been destroyed sucessfully")

if __name__ == "__main__":
    main()