# Transforms

Examples demonstrating `tf2` frame manipulation and robot model visualisation.

## Building
```bash
colcon build --packages-select transforms
source install/setup.bash
```

## Nodes
Run any of the C++ examples to broadcast different transform scenarios:
```bash
ros2 run transforms translation_frame
ros2 run transforms rotation_frame
ros2 run transforms transform_order
ros2 run transforms frame_chains
ros2 run transforms static_dynamic_frame
```

## Launch Files
- `ros2 launch transforms rviz.launch.py` – loads `urdf/diff_drive.urdf` into RViz using the configuration in `config/`.
- `ros2 launch transforms gazebo.launch.py` – spawns the same robot model in Gazebo for simulation.

What you'll learn: creating static and dynamic frames, chaining transforms and visualising URDF models.
