import carla
import os, sys, math
file_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(file_path+'/../')
from utils.udp_server import udp_server

sys.path.append('/opt/carla/PythonAPI/carla')
from agents.navigation.local_planner import LocalPlanner

import random

class mCar:
    def __init__(self, client, spawn_point=None, name = "ego_car"):
        # spawn ego car
        world = client.get_world()
        # ego_bp = world.get_blueprint_library().find('vehicle.carlamotors.european_hgv')
        ego_bp = world.get_blueprint_library().find('vehicle.bmw.grandtourer')
        # ego_bp = world.get_blueprint_library().find('vehicle.dodge.charger_2020')
        # car_select=random.choice(['vehicle.carlamotors.european_hgv','vehicle.tesla.model3','vehicle.dodge.charger_2020'])
        # print(car_select)
        # ego_bp = world.get_blueprint_library().find(car_select)
        # ego_bp.set_attribute('role_name',name)

        if spawn_point is None:
            spawn_points = world.get_map().get_spawn_points()
            # self.spawn_point = spawn_points[171]
            self.spawn_point = spawn_points[0]
        else:
            self.spawn_point = spawn_point
        
        #generate ego car
        self.vehicle = world.try_spawn_actor(ego_bp, self.spawn_point)

        # set the runtime property
        self.run_speed = 120

        # vehicle PHYSICS property
        self.physics = carla.VehiclePhysicsControl()
        # # Create Wheels Physics Control
        # front_left_wheel  = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=70.0, long_stiff_value=1000)
        # front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.0, damping_rate=1.5, max_steer_angle=70.0, long_stiff_value=1000)
        # rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000)
        # rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=3.0, damping_rate=1.5, max_steer_angle=0.0,  long_stiff_value=1000) # Reducing friction increases idle throttle 

        # wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

        # # Change Vehicle Physics Control parameters of the vehicle
        physics_control = self.vehicle.get_physics_control()
        # physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
        # physics_control.max_rpm = 10000
        # physics_control.moi = 1.0
        # physics_control.damping_rate_full_throttle = 0.0
        physics_control.use_gear_autobox = False
        physics_control.gear = 6
        # physics_control.gear_switch_time = 0.5
        # physics_control.clutch_strength = 10
        # physics_control.mass = 10000
        # physics_control.drag_coefficient = 0.25
        # physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
        # physics_control.use_sweep_wheel_collision = True
        # physics_control.wheels = wheels
        self.vehicle.apply_physics_control(physics_control)
        
        # attach imu sensor, gnss sensor, collision sensor to the ego car
        imu_bp = world.get_blueprint_library().find('sensor.other.imu')
        # update rate is 30hz
        # imu_tf = carla.Transform()
        # Define IMU sensor relative transform with respect to the vehicle's center of mass
        imu_relative_transform = carla.Transform(carla.Location(x=0, y=0, z=1), carla.Rotation())

        self.imu = world.spawn_actor(imu_bp, imu_relative_transform, attach_to=self.vehicle)
        

        gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
        # update rate is 30hz
        gnss_tf = carla.Transform()
        self.gnss = world.spawn_actor(gnss_bp, gnss_tf, attach_to=self.vehicle)

        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_tf = carla.Transform(carla.Location())
        self.collision = world.spawn_actor(collision_bp, collision_tf, attach_to=self.vehicle)

        # callback function
        self.imu_data = None
        self.gnss_data = None
        self.collision_data = None
        self.imu.listen(lambda imu_data: self.imu_callback(imu_data))
        self.gnss.listen(lambda gnss_data: self.gnss_callback(gnss_data))
        self.collision.listen(lambda collision_data: self.collision_callback(collision_data))


        # Apply Vehicle Physics Control for the vehicle
        self.localplanner = LocalPlanner(self.vehicle)

        # this part is for matplot juggler
        self._udp_server = udp_server(name=name)

        self.auto = False

    def destroy(self):
        self.imu.destroy()
        self.gnss.destroy()
        self.collision.destroy()
        self.vehicle.destroy()
    
    def get_transform_vec(self):
        return self.vehicle.get_transform()

    def get_focus(self):
        # make spectator follow the ego car
        spectator = self.vehicle.get_world().get_spectator()
        ego_tf = self.get_transform_vec()
        spectator_tf = carla.Transform(ego_tf.location, ego_tf.rotation)
        spectator_tf.location += ego_tf.get_forward_vector() * (-30)
        spectator_tf.location += ego_tf.get_up_vector() * 35

        # change the pitch angle
        spectator_tf.rotation.pitch = -25

        spectator.set_transform(spectator_tf)
    
    def apply_control(self, control):
        # apply the control command on the vehicle
        self.vehicle.apply_control(control)
    
    def set_global_plan(self, route):
        self.localplanner.set_global_plan(route)

    def set_speed(self, speed):
        self.run_speed = speed

    def lp_control_run_step(self,brake = -0.1, throttle = -0.1, run_time = 0):
        self.localplanner.set_speed(self.run_speed)
        control = self.localplanner.run_step()

        if throttle >=0:
            control.throttle = throttle
        if brake >=0:
            control.brake = brake

        self._udp_server.update_control(control)

        if self.imu_data != None and self.gnss_data != None:
            # update imu
            acc = self.imu_data.accelerometer
            gyro = self.imu_data.gyroscope
            acc = [acc.x, acc.y, acc.z]
            gyro = [gyro.x, gyro.y, gyro.z]
            self._udp_server.update_IMU([acc, gyro])

            # update gnss
            lat = self.gnss_data.latitude
            lon = self.gnss_data.longitude
            alt = self.gnss_data.altitude
            self._udp_server.update_GNSS([lat, lon, alt])
            snap = run_time

        # send message
        self._udp_server.update(snap=snap)
        self.apply_control(control)
        return self.localplanner.done()
    
    def imu_callback(self, imu_data):
        self.imu_data = imu_data

    def gnss_callback(self, gnss_data):
        self.gnss_data = gnss_data

    def collision_callback(self, collision_data):
        self.collision_data = collision_data
        # self._udp_server.update_collision(collision_data)

    def trig_autopilot(self):
        if self.auto == True:
            self.auto = False
            self.vehicle.set_autopilot(False)
            print("autopilot off")
        else:
            self.auto = True
            self.vehicle.set_autopilot(True)
            print("autopilot on")

    def update_state(self, run_time=0):
        # update imu if not none 
        if self.imu_data != None:
            return
        acc = self.imu_data.accelerometer 
        gyro = self.imu_data.gyroscope 
        acc = [acc.x, acc.y, acc.z] 
        gyro = [gyro.x, gyro.y, gyro.z] 
        self._udp_server.update_IMU([acc, gyro])

        # update gnss
        lat = self.gnss_data.latitude
        lon = self.gnss_data.longitude
        alt = self.gnss_data.altitude
        self._udp_server.update_GNSS([lat, lon, alt])

        # update snap
        snap = run_time
        self._udp_server.update(snap=snap.frame)
    def get_speed(self):
        return math.sqrt(self._velocity.x**2 + self._velocity.y**2 + self._velocity.z**2)

    @property
    def _velocity(self):
        return self.vehicle.get_velocity()
    
    @property
    def _acceleration(self):
        return self.vehicle.get_acceleration()

    @property
    def _location(self):
        return self.vehicle.get_location()
    
    @property
    def _abs_velocity(self):
        return math.sqrt(self._velocity.x**2 + self._velocity.y**2 + self._velocity.z**2)