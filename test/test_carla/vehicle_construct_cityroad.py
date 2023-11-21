import carla
import cv2
import numpy as np
import random

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.udp_server import udp_server

# params
SPAWN_POINT_NUMBER = 70
IMAGE_HEIGHT = 960 #1080
IMAGE_WIDTH = 960 #1920
WINDOW_NAME = 'RGB Camera bird view'

image_size = (IMAGE_HEIGHT, IMAGE_WIDTH, 4)

class carla_apis():
    def __init__(self, world_name: str="Town10HD_Opt", host='carla_server', port=2000):
        # connect to carla server
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)

        # load world
        self.world = self.client.get_world()
        self.world = self.client.load_world(world_name)
        self.map = self.world.get_map()

        # load blueprint library
        self.blueprint_library = self.world.get_blueprint_library()

        # load actor list
        self.ego_camera = None
        self.ego_vehicle = None

        # image_data is used for restore the camera data
        self.image_data = np.zeros(image_size)

    def spawn_ego_camera(self, vehicle=None):
        # if there is no vehicle, spawn ego vehicle
        if vehicle is None:
            print("no vehicle is given, spawn ego vehicle...")
            vehicle = self.spawn_ego_vehicle()
        if self.ego_camera is not None:
            print("ego camera already exists")
            return

        # spawn camera which is attached to vehicle
        camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        camera_bp.set_attribute('fov', '110')

        # camera_transform = carla.Transform(carla.Location(x=1.5, z=20), carla.Rotation(pitch=-90))
        camera_transform = carla.Transform(carla.Location(x=0.0, z=20.0), carla.Rotation(pitch=-90))


        # attach camera to the vehicle
        self.ego_camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)

        # start callback function
        self.ego_camera.listen(lambda image: self.rgb_callback(image))

    def spawn_ego_vehicle(self, spawn_points_number: int = SPAWN_POINT_NUMBER):
        # spawn ego vehicle
        if self.ego_vehicle is None:
            print("spawn ego vehicle...")
            vehicle_bp = self.blueprint_library.find('vehicle.audi.etron')
            spawn_points = self.world.get_map().get_spawn_points() 
            self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[spawn_points_number])
        else:
            print("ego vehicle already exists")
    
    def ego_vehicle_move(self):
        print("start moving...")
        self.ego_vehicle.set_autopilot(True)

    def terminate(self):
        # destroy actors
        self.ego_camera.stop()
        print("done")

    def spawn_actors(self):
        spawn_points = self.world.get_map().get_spawn_points() 

        for i in range(50):
            # spawn vehicle
            vehicle_bp = random.choice(self.blueprint_library.filter('vehicle')) 
            self.world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
        
        # set autopilot
        for actor in self.world.get_actors().filter('*vehicle*'):
            actor.set_autopilot(True)

    def attach_obstacle(self):
        # attach obstacle sensor to the car
        obstacle_bp = self.blueprint_library.find('sensor.other.obstacle')
        obstacle_bp.set_attribute('hit_radius', '0.5')
        obstacle_bp.set_attribute('distance', '10')

        self.obstacle_sensor = self.world.spawn_actor(obstacle_bp, carla.Transform(carla.Location(x=0.0, z=0.0), carla.Rotation()), attach_to=self.ego_vehicle)
        
        # start callback function
        self.obstacle_sensor.listen(lambda obstacle: self.obstacle_callback(obstacle))

    def obstacle_callback(self, obstacle):
        vehicle_id = obstacle.other_actor.type_id
        # print(vehicle_id)
        if 'vehicle' in vehicle_id:
            print(f'transform: {obstacle.other_actor.type_id}\nframe: {obstacle.frame}')
            print(f'{obstacle.other_actor.get_transform().location}')

    # callback func
    def rgb_callback(self, image):
        self.image_data = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    def get_image_update(self):
        return self.image_data
    

if __name__ == '__main__':
    # launch the world and attach different sensors to the ego vehicle
    carla_api = carla_apis()
    carla_api.spawn_ego_camera()
    carla_api.ego_vehicle_move()
    carla_api.attach_obstacle()
    carla_api.spawn_actors()

    # OpenCV window with initial data
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(WINDOW_NAME, np.zeros(image_size))
    cv2.waitKey(1)

    # inshow the video stream
    while True:
        cv2.imshow(WINDOW_NAME, carla_api.get_image_update())

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    carla_api.terminate()