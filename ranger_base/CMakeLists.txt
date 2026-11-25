cmake_minimum_required(VERSION 3.5)
project(ranger_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ranger_msgs REQUIRED)
find_package(ugv_sdk REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(DEPENDENCIES
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "ranger_msgs"
  "sensor_msgs"
  "tf2_geometry_msgs"
)

# pure motion logic: no ROS, no CAN (unit-tested below)
add_library(ranger_base_logic STATIC src/motion_logic.cpp)
set_target_properties(ranger_base_logic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(ranger_base_node
    src/ranger_base_node.cpp
    src/ranger_messenger.cpp)

target_link_libraries(ranger_base_node ranger_base_logic ugv_sdk)
ament_target_dependencies(ranger_base_node ${DEPENDENCIES})

install(TARGETS ranger_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

# install(DIRECTORY ascent
#   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_logic
    test/test_command_from_twist.cpp
    test/test_odometry_integrator.cpp)
  target_link_libraries(test_motion_logic ranger_base_logic)

  ament_add_gtest(test_replay_recordings test/test_replay_recordings.cpp)
  target_link_libraries(test_replay_recordings ranger_base_logic)
  target_compile_definitions(test_replay_recordings PRIVATE
    REPLAY_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/replay")

  ament_add_gtest(test_contract test/test_contract.cpp)
  target_compile_definitions(test_contract PRIVATE
    MSG_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/msg_jazzy"
    RANGER_MSGS_MSG_DIR="${ranger_msgs_DIR}/../msg")

  # replay tool for the report (not installed)
  add_executable(odom_replay test/odom_replay.cpp)
  target_link_libraries(odom_replay ranger_base_logic)
endif()

ament_package()

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

# add_library(ascent INTERFACE)
# target_include_directories(ascent INTERFACE
# 	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# 	$<INSTALL_INTERFACE:include>)
