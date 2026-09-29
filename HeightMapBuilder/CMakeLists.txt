cmake_minimum_required(VERSION 3.16)
project(height_map_builder VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto -O3 -Wall -Wextra -fPIC")

find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

include(${LCM_USE_FILE})
set(LCM_MSGS_DIR "${CMAKE_CURRENT_LIST_DIR}/../lcm_msgs")

include_directories("include")
include_directories("${LCM_MSGS_DIR}")

file(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")
lcm_wrap_types(CPP_HEADERS cpp_headers ${message_definitions})
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

set(EXEC_SOURCES
  src/main.cpp
  src/HeightMapBuilder.cpp
)
add_executable(${PROJECT_NAME} ${EXEC_SOURCES})
target_link_libraries(${PROJECT_NAME}
  yaml-cpp::yaml-cpp
  ZLIB::ZLIB
)
lcm_target_link_libraries(${PROJECT_NAME} messages-cpp ${LCM_NAMESPACE}lcm)
