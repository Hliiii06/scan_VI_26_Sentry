cmake_minimum_required(VERSION 3.8)
project(sentry_scan_adapter_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(scan_planner_msgs REQUIRED)

add_executable(rm_input_adapter src/rm_input_adapter_node.cpp)
target_include_directories(rm_input_adapter PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(rm_input_adapter ${PCL_LIBRARIES})
ament_target_dependencies(rm_input_adapter
  rclcpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs pcl_conversions)

add_executable(task_adapter src/task_adapter_node.cpp)
ament_target_dependencies(task_adapter
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(reference_path_publisher src/reference_path_publisher_node.cpp)
ament_target_dependencies(reference_path_publisher
  rclcpp geometry_msgs nav_msgs)

install(TARGETS rm_input_adapter task_adapter reference_path_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 点云布局反例（缺字段/错类型/截断/大端/行跨度/offset/count）
  ament_add_gtest(test_cloud_layout_logic test/test_cloud_layout_logic.cpp)
  target_include_directories(test_cloud_layout_logic PRIVATE src)
  ament_target_dependencies(test_cloud_layout_logic
    rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs std_msgs
    tf2 tf2_ros tf2_geometry_msgs pcl_conversions)
  target_link_libraries(test_cloud_layout_logic ${PCL_LIBRARIES})
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
