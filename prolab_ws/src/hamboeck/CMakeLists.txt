cmake_minimum_required(VERSION 3.0.2)
project(hamboeck)

# Set C++11 standard
add_compile_options(-std=c++11)

# Find required ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Define catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Define the executable that uses the header files
add_executable(${PROJECT_NAME}_node src/mcl/mcl.cpp)
add_executable(publish_poses src/publish_poses/publish_poses.cpp)

# Link the executable with the necessary libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(publish_poses ${catkin_LIBRARIES})

add_dependencies(publish_poses ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

