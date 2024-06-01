cmake_minimum_required(VERSION 3.8)
project(transforms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(translation_frame src/translation_frame.cpp)
ament_target_dependencies(translation_frame rclcpp tf2_ros geometry_msgs)

add_executable(rotation_frame src/rotation_frame.cpp)
ament_target_dependencies(rotation_frame rclcpp tf2_ros geometry_msgs)

add_executable(transform_order src/transform_order.cpp)
ament_target_dependencies(transform_order rclcpp tf2_ros geometry_msgs)

add_executable(frame_chains src/frame_chains.cpp)
ament_target_dependencies(frame_chains rclcpp tf2_ros geometry_msgs)

add_executable(static_dynamic_frame src/static_dynamic_frame.cpp)
ament_target_dependencies(static_dynamic_frame rclcpp tf2_ros geometry_msgs)


install(TARGETS
translation_frame rotation_frame transform_order frame_chains static_dynamic_frame
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
