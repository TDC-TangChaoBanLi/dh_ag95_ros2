cmake_minimum_required(VERSION 3.8)
project(dh_ag95_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dh_ag95_msgs REQUIRED)

if(NOT TARGET dh_ag95_core)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/dh_ag95_driver ${CMAKE_CURRENT_BINARY_DIR}/dh_ag95_core_driver)
endif()

add_executable(dh_ag95_node src/ag95_node.cpp)
target_link_libraries(dh_ag95_node dh_ag95_core)
ament_target_dependencies(dh_ag95_node rclcpp rclcpp_action std_msgs dh_ag95_msgs)

target_include_directories(dh_ag95_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/dh_ag95_driver/include)

install(TARGETS dh_ag95_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
