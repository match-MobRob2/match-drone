cmake_minimum_required(VERSION 3.8)
project(marvin_nav)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

find_package(nav_msgs REQUIRED)

set(DEPS rclcpp sensor_msgs geometry_msgs nav_msgs
         tf2_ros tf2_eigen pcl_conversions pcl_ros octomap_msgs)

add_executable(mapper_planner_node src/mapper_planner_node.cpp)

target_include_directories(mapper_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(mapper_planner_node
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(mapper_planner_node ${DEPS})

install(TARGETS mapper_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
