cmake_minimum_required(VERSION 3.8)
project(robot_mode_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)  # suppress -Wdev warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto -DEIGEN_STACK_ALLOCATION_LIMIT=0 -O3 -fPIC")

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mors_ros_msgs REQUIRED)
find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

include(${LCM_USE_FILE})
set(LCM_MSGS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lcm_msgs")
include_directories("${LCM_MSGS_DIR}")

FILE(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")

# Generate headers from message definition
lcm_wrap_types(CPP_HEADERS cpp_headers
  ${message_definitions})

# Create library from all the messages
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

add_executable(robot_mode_controller 
    src/robot_mode_controller.cpp
    src/lcm_data_exchange.cpp
    src/action1.cpp
    src/action2.cpp
    )

ament_target_dependencies(robot_mode_controller 
    rclcpp 
    std_msgs 
    geometry_msgs
    mors_ros_msgs)

target_link_libraries(robot_mode_controller Eigen3::Eigen)
target_link_libraries(robot_mode_controller yaml-cpp::yaml-cpp)
lcm_target_link_libraries(robot_mode_controller messages-cpp ${LCM_NAMESPACE}lcm)

install(TARGETS
  robot_mode_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
