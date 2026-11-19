cmake_minimum_required(VERSION 3.0.2)
project(gbplanner)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(adaptive_obb REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${planner_common_INCLUDE_DIRS}
  /home/nadaabbas/matplotplusplus/include
)


# Include Matplot++
add_subdirectory(/home/nadaabbas/matplotplusplus /home/nadaabbas/matplotplusplus/build)

cs_add_library(
  ${PROJECT_NAME}
  src/gbplanner.cpp
  src/rrg.cpp
  src/gbplanner_rviz.cpp
)

add_executable(trajectory_transformer src/trajectory_transformer.cpp)
target_link_libraries(trajectory_transformer
  ${catkin_LIBRARIES}  # This ensures all ROS core libraries are linked
  ${Boost_LIBRARIES}   # Only needed if you actually use Boost in your code
)

add_executable(p2p src/p2p.cpp)
target_link_libraries(p2p
  ${catkin_LIBRARIES}  # This ensures all ROS core libraries are linked
  ${Boost_LIBRARIES}   # Only needed if you actually use Boost in your code
)

add_executable(dta_node src/dta_node.cpp)
target_link_libraries(dta_node
  ${catkin_LIBRARIES}  # This ensures all ROS core libraries are linked
  ${Boost_LIBRARIES}   # Only needed if you actually use Boost in your code
)

add_executable(Dynamic_Auction src/Dynamic_Auction.cpp)
target_link_libraries(Dynamic_Auction
  ${catkin_LIBRARIES}  # This ensures all ROS core libraries are linked
  ${Boost_LIBRARIES}   # Only needed if you actually use Boost in your code
)

add_executable(plots_visualizer_node src/plots_visualizer_node.cpp)
target_link_libraries(plots_visualizer_node
  ${catkin_LIBRARIES}
  matplot
)

cs_add_executable(gbplanner_node src/gbplanner_ros_node.cpp)
target_link_libraries(gbplanner_node ${PROJECT_NAME} ${planner_common_LIBRARIES})

cs_install()
cs_export(LIBRARIES)
