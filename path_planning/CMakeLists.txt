cmake_minimum_required(VERSION 3.5)
project(path_planning)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include ${PCL_INCLUDE_DIRS})

# Add executable
add_executable(path_planning src/path_planning.cpp)
ament_target_dependencies(path_planning
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  geometry_msgs
  visualization_msgs
  std_msgs
  Eigen3
)

target_include_directories(path_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)


# Link PCL libraries
target_link_libraries(path_planning ${PCL_LIBRARIES})

# Install targets
install(TARGETS
  path_planning
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
