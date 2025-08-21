# Robot Math

ROS 2 package showcasing basic control equations with `turtlesim`.

## Building
```bash
colcon build --packages-select robot_math
source install/setup.bash
```
Ensure `turtlesim` is running in another terminal:
```bash
ros2 run turtlesim turtlesim_node
```

## Nodes
- `circle_motion_node` – drives a circular path based on linear velocity and radius:
  ```bash
  ros2 run robot_math circle_motion_node <linear_vel> <radius>
  ```
- `go_To_Goal_node` – moves the turtle toward a specified goal pose:
  ```bash
  ros2 run robot_math go_To_Goal_node <x> <y> <theta>
  ```

Both nodes publish to `/turtle1/cmd_vel` and demonstrate angular velocity and go‑to‑goal control.

What you'll learn: applying mathematical formulas to control a robot in ROS 2.
