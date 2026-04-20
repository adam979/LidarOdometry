cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

# C++17 required for structured bindings (auto & [si, ti])
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Core library (Eigen only)
add_library(lidar_odometry_core
  src/preprocessor.cpp
  src/icp.cpp
)

target_include_directories(lidar_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# ── ROS2 node executable ───────────────────────────────────────────
add_executable(odometry_node
  src/odometry_node.cpp
)

target_include_directories(odometry_node PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(odometry_node
  lidar_odometry_core
)

ament_target_dependencies(odometry_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install targets
install(TARGETS odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lidar_odometry_core
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()