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
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

set(DEPS rclcpp sensor_msgs geometry_msgs nav_msgs std_srvs std_msgs
         tf2_ros tf2_eigen pcl_conversions pcl_ros octomap_msgs)

add_executable(mapper_planner_node src/mapper_planner_node.cpp)
add_executable(lidar_relocalization_node src/lidar_relocalization_node.cpp)

foreach(target mapper_planner_node lidar_relocalization_node)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(${target}
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(${target} ${DEPS})
endforeach()

# Offline-Tool, kein ROS-Node: Pose-Graph-Optimierung einer Mapping-Aufnahme
add_executable(map_optimizer src/map_optimizer.cpp)
target_include_directories(map_optimizer PUBLIC ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(map_optimizer gtsam yaml-cpp ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS mapper_planner_node lidar_relocalization_node map_optimizer
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
