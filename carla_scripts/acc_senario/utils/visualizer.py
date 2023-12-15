import carla
import numpy as np
import cv2
import queue

def visualize_waypoint(client, waypoints, resolution):
    world = client.get_world()
    location_temp = None
    for waypoint in waypoints:
        # get forward vector
        if location_temp is not None:
            vector = (waypoint[0].transform.location - location_temp)/resolution
            world.debug.draw_arrow(waypoint[0].transform.location + carla.Location(z=0.5), waypoint[0].transform.location + carla.Location(z=0.5) + vector, life_time=100)
        location_temp = waypoint[0].transform.location

# def camera(client, vehicle):
#     world = client.get_world()
#     camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
#     camera_bp.set_attribute('image_size_x', '1920')
#     camera_bp.set_attribute('image_size_y', '1080')
#     camera_bp.set_attribute('fov', '110')
#     camera_bp.set_attribute('sensor_tick', '0.02')
#     camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
#     camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
#     return camera

class camera(object):
    def __init__(self, client, vehicle):
        self.HIGHT = 720
        self.WIDTH = 720

        self.world = client.get_world()
        self.camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera_bp.set_attribute('image_size_x', f"{self.WIDTH}")
        self.camera_bp.set_attribute('image_size_y', f"{self.HIGHT}")
        self.camera_bp.set_attribute('fov', '110')
        self.camera_bp.set_attribute('sensor_tick', '0.02')
        self.camera_transform = carla.Transform(carla.Location(x=-10, z=5), carla.Rotation(pitch=-15))
        self.camera = self.world.spawn_actor(self.camera_bp, self.camera_transform, attach_to=vehicle)
        self.image = queue.Queue()
        self.camera.listen(lambda image: self.image_cb(image))

    def image_cb(self, image):
        image = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))
        self.image.put(image)

    def show_image(self):
        if self.image is not None:
            cv2.imshow("spectator", self.image.get())
            cv2.waitKey(1)