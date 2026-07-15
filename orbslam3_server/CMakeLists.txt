cmake_minimum_required(VERSION 3.8)
project(orbslam3_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orbslam3_multi REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# F1B: `global_map_server` es el servidor minimo activo. Debe compilar sin
# enlazar ORB_SLAM3/g2o para demostrar que la recepcion ROS de OrbMap queda
# separada del servidor monolitico congelado.
add_executable(global_map_server src/global_map_server.cpp)

target_include_directories(global_map_server PRIVATE
  include
)

add_executable(global_pose_corrector src/global_pose_corrector.cpp)

target_include_directories(global_pose_corrector PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(
  global_map_server
  rclcpp
  geometry_msgs
  sensor_msgs
  orbslam3_msgs
  orbslam3_multi
)

ament_target_dependencies(
  global_pose_corrector
  rclcpp
  orbslam3_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(
  TARGETS global_map_server global_pose_corrector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
