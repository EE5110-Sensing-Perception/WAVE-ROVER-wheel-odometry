cmake_minimum_required(VERSION 3.8)
  project(odom_wheel)  # Replace with your package name

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(odometry_estimator src/odom_wheel.cpp)

# Link dependencies
ament_target_dependencies(odometry_estimator
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Install the executable
install(TARGETS odometry_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files (if they exist)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Install package.xml (required for ROS2 packaging)
ament_package()

