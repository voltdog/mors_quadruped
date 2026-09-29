cmake_minimum_required(VERSION 3.8)
project(robot_state_viewer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(lcm REQUIRED)
find_package(ZLIB REQUIRED)

include(${LCM_USE_FILE})
set(LCM_MSGS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lcm_msgs")
include_directories("${LCM_MSGS_DIR}")

file(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")
lcm_wrap_types(CPP_HEADERS cpp_headers ${message_definitions})
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

add_executable(robot_state_viewer_node
  src/robot_state_viewer_node.cpp
)

ament_target_dependencies(robot_state_viewer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

target_link_libraries(robot_state_viewer_node
  yaml-cpp::yaml-cpp
  ZLIB::ZLIB
)

lcm_target_link_libraries(robot_state_viewer_node
  messages-cpp
  ${LCM_NAMESPACE}lcm
)

install(TARGETS
  robot_state_viewer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../common/urdf/mors.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
