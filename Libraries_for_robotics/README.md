# Libraries for Robotics

This folder contains small programs that verify the installation of common robotics libraries.

## OMPL
Run the provided test program to check the Open Motion Planning Library:
```bash
# Rebuild if needed
# g++ ompl.cpp -o ompl $(pkg-config --cflags --libs ompl)
./ompl
```
Expected output: `OMPL OK`.

## PCL
Test the Point Cloud Library:
```bash
# g++ pcl.cpp -o pcl $(pkg-config --cflags --libs pcl_common)
./pcl
```
Expected output: `PCL OK`.

## OpenCV
`opencv.py` loads *humanoid_robot.jpeg*, converts it to grayscale and displays both images.
```bash
python opencv.py
```
Edit `image_path` in the script if the image is stored elsewhere.

## PyTorch
`pytorch.py` classifies *humanoid_robot.jpeg* with a pre‑trained ResNet‑18 model and prints the top label:
```bash
python pytorch.py
```
This requires PyTorch and torchvision.

## CHAMP Integration Example
`champ_libraries_integration` is a ROS 2 package combining OpenCV, PyTorch, PCL and OMPL in a single workspace.
Build inside your ROS 2 workspace and run the nodes:
```bash
colcon build --packages-select champ_libraries_integration
source install/setup.bash
ros2 run champ_libraries_integration opencv_pytorch
ros2 run champ_libraries_integration pcl_ompl
```
These nodes subscribe to camera or point‑cloud topics and demonstrate processing with the above libraries.

What you learn: how to verify and combine popular robotics libraries within ROS 2 projects.
