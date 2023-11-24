import carla

class mCar:
    def __init__(self, client, spawn_point=None):
        # spawn ego car
        world = client.get_world()
        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')

        if spawn_point is None:
            spawn_points = world.get_map().get_spawn_points()
            self.spawn_point = spawn_points[171]
        else:
            self.spawn_point = spawn_point
        
        #generate ego car
        self.vehicle = world.try_spawn_actor(ego_bp, self.spawn_point)

        # vehicle PHYSICS property
        self.physics = carla.VehiclePhysicsControl()
        # Create Wheels Physics Control
        front_left_wheel  = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=70.0, long_stiff_value=1000)
        front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=70.0, long_stiff_value=1000)
        rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000)
        rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000) # Reducing friction increases idle throttle 

        wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

        # Change Vehicle Physics Control parameters of the vehicle
        physics_control = self.vehicle.get_physics_control()
        physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        physics_control.max_rpm = 10000
        physics_control.moi = 1.0
        physics_control.damping_rate_full_throttle = 0.0
        physics_control.use_gear_autobox = True
        physics_control.gear_switch_time = 0.5
        physics_control.clutch_strength = 10
        physics_control.mass = 10000
        physics_control.drag_coefficient = 0.25
        physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
        physics_control.use_sweep_wheel_collision = True
        physics_control.wheels = wheels

        # Apply Vehicle Physics Control for the vehicle
        self.vehicle.apply_physics_control(physics_control)

    def destroy(self):
        self.vehicle.destroy()
    
    def get_transform_vec(self):
        return self.vehicle.get_transform()

    def get_focus(self):
        # make spectator follow the ego car
        spectator = self.vehicle.get_world().get_spectator()
        ego_tf = self.get_transform_vec()
        spectator_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
        spectator_tf.location += ego_tf.get_forward_vector() * (-10)
        spectator_tf.location += ego_tf.get_up_vector() * 3
        spectator.set_transform(spectator_tf)