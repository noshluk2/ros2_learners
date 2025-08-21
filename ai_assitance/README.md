# AI Assistance

ROS 2 Python package providing simple motion behaviours and a Gazebo bringup.

## Building
Add this package to your ROS 2 workspace and build:
```bash
colcon build --packages-select ai_assitance
source install/setup.bash
```

## Nodes
- `my_circle_movement` – publishes velocity commands that drive a robot in a circle.
  ```bash
  ros2 run ai_assitance my_circle_movement
  ```
- `obstacle_avoiding_node` – subscribes to `/scan` and steers away when obstacles are closer than 0.5 m.
  ```bash
  ros2 run ai_assitance obstacle_avoiding_node
  ```

## Gazebo Launch
Spawn the sample `dolly` robot in Gazebo:
```bash
ros2 launch ai_assitance gazebo_bringup.launch.py
```
This launch file runs the state publishers and uses `gazebo_ros` to spawn the URDF model.

What you'll learn: writing basic ROS 2 nodes, publishing velocity commands and launching a robot model in simulation.
