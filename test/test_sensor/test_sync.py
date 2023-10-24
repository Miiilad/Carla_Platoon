import carla
import numpy as np
import cv2

HOST = 'carla_server'
PORT = 2000

client = carla.Client(HOST, PORT) # connect to the server
client.set_timeout(10.0)
world = client.get_world()

# Load the desired map
client.load_world("Town10HD_Opt")

# Set synchronous mode settings
# new_settings = world.get_settings()
# new_settings.synchronous_mode = True
# new_settings.fixed_delta_seconds = 0.05
# world.apply_settings(new_settings) 

client.reload_world(False) # reload map keeping the world settings

# Set up the traffic manager
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)
traffic_manager.set_random_device_seed(200) # define TM seed for determinism

# Spawn your vehicles, pedestrians, etc.
bp_lib = world.get_blueprint_library() 
# Add the ego vehicle
spawn_points = world.get_map().get_spawn_points() 
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])
# Add camera sensor
camera_bp = bp_lib.find('sensor.camera.rgb') 
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

# Get the attributes from the camera
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()
# All sensor callbacks
def rgb_callback(image, data_dict):
    data_dict['rgb_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
               'collision': False,
               'lane_invasion': False,
               'gnss': [0,0],
               'obstacle': [],
               'imu': {
                    'gyro': carla.Vector3D(),
                    'accel': carla.Vector3D(),
                    'compass': 0
                }}
# OpenCV window with initial data
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera', sensor_data['rgb_image'])
cv2.waitKey(1)

# Start sensors recording data
camera.listen(lambda image: rgb_callback(image, sensor_data))

# Simulation loop
while True:
    # Your code
    # Display RGB image with imshow
    cv2.imshow('RGB Camera', sensor_data['rgb_image'])
    world.tick()
    
    # Break the loop if the user presses the Q key
    if cv2.waitKey(1) == ord('q'):
        break
    # world.tick()

# Close the OpenCV display window when the game loop stops and stop sensors
camera.stop()
cv2.destroyAllWindows()