cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mynodecpp src/my_2node.cpp)
ament_target_dependencies(mynodecpp rclcpp)

install (TARGETS
mynodecpp
DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
