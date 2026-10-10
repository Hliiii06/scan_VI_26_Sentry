cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)

add_library(path_searching src/dyn_a_star.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_searching plan_env rclcpp)
target_link_libraries(path_searching Eigen3::Eigen)

if(BUILD_TESTING)
  enable_testing()
  add_executable(search_regression test/search_regression.cpp)
  target_link_libraries(search_regression path_searching)
  ament_target_dependencies(search_regression plan_env rclcpp)
  add_test(NAME search_regression COMMAND search_regression --ros-args --params-file
    ${CMAKE_CURRENT_SOURCE_DIR}/../plan_manage/config/sentry_planner.yaml)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_ros_home)
  set_tests_properties(search_regression PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=73;ROS_LOCALHOST_ONLY=1;ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/test_ros_home")
endif()

install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 plan_env rclcpp)
ament_package()
