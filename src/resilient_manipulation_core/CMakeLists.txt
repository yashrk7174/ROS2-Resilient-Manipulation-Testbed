cmake_minimum_required(VERSION 3.8)
project(resilient_manipulation_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
# ============================================================
#  Panda State Monitor
# ============================================================

add_executable(
  panda_state_monitor
  src/panda_state_monitor.cpp
)

target_include_directories(
  panda_state_monitor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  panda_state_monitor
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  panda_state_monitor
  rclcpp
  sensor_msgs
)

# ============================================================
#  Joint-Space Planner
# ============================================================

add_executable(
  joint_space_planner
  src/joint_space_planner.cpp
)

target_include_directories(
  joint_space_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  joint_space_planner
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  joint_space_planner
  rclcpp
  moveit_ros_planning_interface
)
# ============================================================
#  Cartesian Path Planner
# ============================================================

add_executable(
  cartesian_path_planner
  src/cartesian_path_planner.cpp
)

target_include_directories(
  cartesian_path_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  cartesian_path_planner
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  cartesian_path_planner
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
# ============================================================
#  Collision-Aware Planner
# ============================================================

add_executable(
  obstacle_avoidance_planner
  src/obstacle_avoidance_planner.cpp
)

target_include_directories(
  obstacle_avoidance_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  obstacle_avoidance_planner
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  obstacle_avoidance_planner
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
# ============================================================
#     Baseline Manipulation Mission
# ============================================================

add_executable(
  manipulation_mission
  src/manipulation_mission.cpp
)

target_include_directories(
  manipulation_mission
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  manipulation_mission
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  manipulation_mission
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)
# ============================================================
#  Pose Uncertainty Injector
# ============================================================

add_executable(
  pose_uncertainty_injector
  src/pose_uncertainty_injector.cpp
)

target_include_directories(
  pose_uncertainty_injector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  pose_uncertainty_injector
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  pose_uncertainty_injector
  rclcpp
  geometry_msgs
)



# Install executables
install(
  TARGETS
    panda_state_monitor
    joint_space_planner
    cartesian_path_planner
    obstacle_avoidance_planner
    manipulation_mission
    pose_uncertainty_injector
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
