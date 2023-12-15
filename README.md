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

## velocity control of the ego vehicle

In this file [carla_scripts/control/ego_control_loop.py](https://github.com/Mafumaful/Carla_ROS2/blob/main/carla_scripts/control/ego_control_loop.py), I implemented the velocity control of the ego vehicle. The [controller](https://github.com/Mafumaful/Carla_ROS2/blob/main/carla_scripts/acc_senario/utils/controller.py) is a pid controller with feedforward control.

The control loop is shown below:

+ loop_5ms_loop: the loop is used to update the state of ego vehicle with 5ms frequency
+ loop_10ms_loop: the loop is used to update the pid controller with 10ms frequency (inner loop)
+ loop_100ms_loop: the loop is used to update the throttle with 100ms frequency (outer loop)

![ego control loop](./pictures/velocity%20pid%20feedforward%20control.png)

As we can see in the picture the velocity of the vehicle can always track the reference velocity.

## added sync mode and camera(two ways to avoid "shaking" problem)

In this update, I added camera which attached to the ego vehilce to avoid "shaking" problems. And I tried the sync mode to avoid the "shaking" problem. The result is shown below:

![sync mode](./pictures/sync%20mode%20plot.png)

As you can see in the picture the IMU data becomes more stable compared with the async mode.

To use the sync mode, we can use the following command:

```bash
# in one terminal
python /home/docker/carla_scripts/acc_senario/sync_master/slave.py
# in another terminal
python /home/docker/carla_scripts/control/ego_control_loop_sync.py
```
