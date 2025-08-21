# Nodes

C++ examples demonstrating core ROS 2 concepts: publishers, subscribers, timers and launch files.

## Building
```bash
colcon build --packages-select nodes
source install/setup.bash
```

## Publisher and Subscriber
Start the basic talker and listener:
```bash
ros2 run nodes publisher
ros2 run nodes subscriber
```
The publisher sends messages on `topic_1` and `topic_2`; the subscriber logs the received strings and integers.

## Laser Scan Publisher
```bash
ros2 run nodes laser_publisher
```
Publishes a synthetic `sensor_msgs/LaserScan` on `/scan`.

## Launch Examples
- `ros2 launch nodes nodes.launch.py` – runs two `turtlesim` nodes and mirrors motion using the `mimic` node.
- `ros2 launch nodes map_launch.py` – starts a `nav2_map_server` using the images in `maps/`.

What you'll learn: building and launching simple ROS 2 nodes and publishing custom messages.
