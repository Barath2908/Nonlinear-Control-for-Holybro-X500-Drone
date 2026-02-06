cmake_minimum_required(VERSION 3.8)
project(vio_mavlink_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Optional: MAVROS messages (if using MAVROS interface)
find_package(mavros_msgs QUIET)

# MAVLink - download if not found
find_path(MAVLINK_INCLUDE_DIR
  NAMES mavlink/common/mavlink.h
  PATHS
    /usr/local/include
    /usr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/mavlink
)

if(NOT MAVLINK_INCLUDE_DIR)
  message(STATUS "MAVLink not found, downloading...")
  include(FetchContent)
  FetchContent_Declare(
    mavlink
    GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(mavlink)
  set(MAVLINK_INCLUDE_DIR ${mavlink_SOURCE_DIR})
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIR}
)

# ============== MAVLink Bridge Node ==============
add_executable(mavlink_bridge_node
  src/mavlink_bridge_node.cpp
)

target_include_directories(mavlink_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mavlink_bridge_node
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============== Sliding Mode Controller Node ==============
if(mavros_msgs_FOUND)
  add_executable(sliding_mode_controller
    src/sliding_mode_controller.cpp
  )

  target_include_directories(sliding_mode_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(sliding_mode_controller
    rclcpp
    rclcpp_components
    nav_msgs
    geometry_msgs
    std_msgs
    mavros_msgs
  )

  target_link_libraries(sliding_mode_controller
    Eigen3::Eigen
  )

  install(TARGETS sliding_mode_controller
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "mavros_msgs not found, sliding_mode_controller will not be built")
endif()

# ============== Installation ==============

# Install executables
install(TARGETS mavlink_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS
  scripts/stereo_calibration.py
  scripts/ekf_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============== Testing ==============
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_mavlink_bridge
    test/test_mavlink_bridge.cpp
  )
  
  target_include_directories(test_mavlink_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(test_mavlink_bridge
    rclcpp
    nav_msgs
    geometry_msgs
  )
endif()

ament_package()
