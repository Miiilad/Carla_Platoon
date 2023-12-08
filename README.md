# start

First, you should build the project, you can build the project offline or pull the image online:

## build the project

```bash
# in the workspace, set the environment variable
chmod +x dev_config.sh
./dev_config.sh

# build the project offline, option 1
docker compose build
# pull the environement online, option 2
docker compose pull
```

Use control + c to quit the environment.

## run the projectd

```bash
docker compose up -d
```

## notes

In the docker carla-ros2-bridge, we can specify the host number using "host='carla_server'"

```bash
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:='carla_server'
```

if you want to see the example, you can use this(you can change it in the docker file):

```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py host:='carla_server'
```

## tutorial

### add birdview camera

```python
import carla
import cv2
import numpy as np

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# import utils
from utils.misc import get_open_port

# params
IMAGE_HEIGHT = 540 #1080
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
        camera_transform = carla.Transform(carla.Location(x=1.5, z=20), carla.Rotation(pitch=-90))
        self.ego_camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)
        # self.ego_camera.listen(lambda image: rgb_callback(self.image_data, image))
        self.ego_camera.listen(lambda image: self.rgb_callback(image))

    def spawn_ego_vehicle(self, spawn_points_number: int = 78):
        # spawn ego vehicle
        if self.ego_vehicle is None:
            print("spawn ego vehicle...")
            vehicle_bp = self.blueprint_library.find('vehicle.audi.etron')
            spawn_points = self.world.get_map().get_spawn_points() 
            self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[spawn_points_number])
        else:
            print("ego vehicle already exists")

    def terminate(self):
        # destroy actors
        self.ego_camera.stop()
        print("done")

    # callback func
    def rgb_callback(self, image):
        self.image_data = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

if __name__ == '__main__':
    carla_api = carla_apis()
    carla_api.spawn_ego_camera()

    # OpenCV window with initial data
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(WINDOW_NAME, np.zeros(image_size))
    cv2.waitKey(1)

    while True:
        cv2.imshow(WINDOW_NAME, carla_api.image_data)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    carla_api.terminate()
    cv2.destroyALLWindows()
```

### visulization

to visualize the project we can use carlaviz

```bash
docker pull mjxu96/carlaviz:0.9.13
docker run -it --network="host" mjxu96/carlaviz:0.9.13 \
  --simulator_host localhost \
  --simulator_port 2000
```

## check the spawn points

We can check the spawn points with birdview camera using the following command:

```bash
python carla_scripts/examples/spawn_point.py
```

## ROS2 development (with ros2 bridge)

```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py host:=carla_server
```

You can also use rviz2 to visualize the data:

```bash
rviz2
```

## Acceleraion control


To implement the acceleration control, I tested the car with different throttle in different speed. The result is shown below:

![throttle speed map tested in carla](./pictures/speed%20throttle%20acc%20tested%20in%20carla.png)

to make the neural network keep far away from the bad data as shown in picture below, I applied the following data filter:

![bad data](./pictures/Accel%20Throttle%20Speed%20map%20filtered.png)

```python
# filt the data
# Apply the first condition
condition1 = (data['acceleration'] < 20) & (data['speed'].abs() < 0.1)
# Apply the second condition
condition2 = data['speed'] > 30
# Combine conditions with OR (|) since rows should be deleted if they meet either condition
combined_condition = condition1 | condition2
# Invert the combined condition and use it to filter the DataFrame
data = data[~combined_condition]
```

To further use the throttle to control the acceleration, I created $2 \times10 \times 1$ linear [neural network](https://github.com/Mafumaful/Carla_ROS2/blob/main/carla_scripts/neural/model.py) to map the throttle to the acceleration. The result is shown below:

![neural network throttle speed map](./pictures/Accel%20Throttle%20Speed%20map.png)
