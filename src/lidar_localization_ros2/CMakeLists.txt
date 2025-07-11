cmake_minimum_required(VERSION 3.5)
project(lidar_localization_ros2)

# fix pcl policy warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


add_library(lidar_localization_component SHARED
src/lidar_localization_component.cpp
)

ament_target_dependencies(lidar_localization_component
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
target_link_libraries(lidar_localization_node
  lidar_localization_component
  ${PCL_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

ament_export_libraries(lidar_localization_component)

install(TARGETS
  lidar_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
