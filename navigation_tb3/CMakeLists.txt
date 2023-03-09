cmake_minimum_required(VERSION 3.8)
project(navigation_tb3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
# New Eidtions
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
#

add_executable(pub_occupancy_grid src/pub_occupancy_grid.cpp)
ament_target_dependencies(pub_occupancy_grid rclcpp nav_msgs)

install(TARGETS
  pub_occupancy_grid
  DESTINATION lib/${PROJECT_NAME})

install( DIRECTORY launch/
         DESTINATION share/${PROJECT_NAME}/launch)
install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config)


ament_python_install_package(${PROJECT_NAME})
# Python Executables into Lib

install(PROGRAMS scripts/single_goal_nav.py
        DESTINATION lib/${PROJECT_NAME})

        install(PROGRAMS scripts/multi_waypoints.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
