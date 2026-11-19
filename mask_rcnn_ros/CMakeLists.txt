cmake_minimum_required(VERSION 3.5)
project(mask_rcnn_ros)

# Specify the C++ Standard
add_definitions(-std=c++14)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  message_generation
)

# Setup Python (if you're using Python nodes/scripts)
catkin_python_setup()
add_service_files(
  FILES
  VisitedPoint.srv
  # Add other service files here as needed
)

# Declare ROS messages and services
add_message_files(
  FILES
  Result.msg
  CenterAndRadius.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(image_in_sync_node scripts/image_in_sync_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(image_in_sync_node ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS image_in_sync_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python scripts
install(PROGRAMS
  scripts/mask_rcnn_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other directories if necessary
install(DIRECTORY
  msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
