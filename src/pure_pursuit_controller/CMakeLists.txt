cmake_minimum_required(VERSION 3.5)
project(pure_pursuit_controller)

# fix pcl policy warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS})

# PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Pure Pursuit Controller executable
add_executable(pure_pursuit_controller
  src/pure_pursuit_controller.cpp
  src/pure_pursuit_main.cpp
)

ament_target_dependencies(pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Waypoint Recorder executable
add_executable(waypoint_recorder
  src/waypoint_recorder.cpp
)

ament_target_dependencies(waypoint_recorder
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(waypoint_recorder yaml-cpp)

# Path Loader executable
add_executable(path_loader
  src/path_loader.cpp
)

ament_target_dependencies(path_loader
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

target_link_libraries(path_loader yaml-cpp)

# Emergency Braker executable
add_executable(emergency_braker
  src/emergency_braker.cpp
)

ament_target_dependencies(emergency_braker
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ackermann_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(emergency_braker ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  pure_pursuit_controller
  waypoint_recorder
  path_loader
  emergency_braker
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
