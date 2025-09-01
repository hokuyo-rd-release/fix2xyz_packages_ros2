cmake_minimum_required(VERSION 3.5)
project(fix2xyz)

# ROS2の依存関係を見つける
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fix_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PROJ REQUIRED)


include_directories(
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(fix2xyz_node
  src/fix2xyz_node.cpp
  src/fix2xyz_node_lib.cpp
  src/fix_xyz_transform.cpp
)

add_executable(xyz2fix_node
  src/xyz2fix_node.cpp
  src/xyz2fix_node_lib.cpp
  src/fix_xyz_transform.cpp
)

add_executable(tf_test
  src/tf_test.cpp
)

ament_target_dependencies(fix2xyz_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  fix_msgs
  PROJ
  Eigen3
)

ament_target_dependencies(xyz2fix_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  fix_msgs
  PROJ
  Eigen3
)

ament_target_dependencies(tf_test
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  fix_msgs
  PROJ
  Eigen3
)

install(TARGETS
  fix2xyz_node
  xyz2fix_node
  tf_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()