cmake_minimum_required(VERSION 3.8)
project(test_moveit_cpp_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp rclcpp_action moveit_ros_planning_interface tf2_ros) 

install (TARGETS
test_moveit
DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
