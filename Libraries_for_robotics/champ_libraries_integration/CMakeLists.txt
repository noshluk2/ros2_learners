cmake_minimum_required(VERSION 3.14)
project(champ_libraries_integration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(ompl REQUIRED)

# ---- C++ node ----
add_executable(pcl_ompl src/pcl_ompl.cpp)
target_include_directories(pcl_ompl PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcl_ompl ${PCL_LIBRARIES} ${OMPL_LIBRARIES})
ament_target_dependencies(pcl_ompl rclcpp sensor_msgs pcl_conversions)

install(TARGETS pcl_ompl
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ---- Python node ----
install(PROGRAMS
  src/opencv_pytorch.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
