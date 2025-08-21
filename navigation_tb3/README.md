# Navigation TB3

TurtleBot3 navigation demos using the Nav2 stack.

## Building
```bash
colcon build --packages-select navigation_tb3
source install/setup.bash
```

## Launch
- `ros2 launch navigation_tb3 mapping.launch.py` – start SLAM mapping and generate `tb3_map`.
- `ros2 launch navigation_tb3 navigation.launch.py` – bring up Nav2 using the provided map and parameters.

## Nodes and Scripts
- `ros2 run navigation_tb3 pub_occupancy_grid` – publish an occupancy grid from `config/tb3_map.yaml`.
- `ros2 run navigation_tb3 single_goal_nav` – send a single goal pose using `nav2_simple_commander`.
- `ros2 run navigation_tb3 multi_waypoints` – patrol a list of waypoints.

What you'll learn: configuring Nav2 for TurtleBot3, mapping an environment and commanding navigation goals from Python.
