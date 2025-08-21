# Point Cloud Perception

Point Cloud Library (PCL) examples for filtering, segmentation and mapping.

## Building
```bash
colcon build --packages-select point_cloud_perception
source install/setup.bash
```

## Executables
Run the compiled demos:
```bash
ros2 run point_cloud_perception voxeling       # voxel grid downsampling
ros2 run point_cloud_perception planner_seg    # plane segmentation
ros2 run point_cloud_perception clustering     # cylinder extraction
```
`clustering` expects a PCD file in `point_clouds/` and writes extracted clouds back to the same directory.

## Launch Files
- `ros2 launch point_cloud_perception 3d_depth_mapping_rtab.launch.py`
- `ros2 launch point_cloud_perception rtab_mapping.launch.py`

These launches start RTAB‑Map for 3D mapping and depth processing.

What you'll learn: how to process point clouds in ROS 2 using PCL and integrate them with mapping tools.
