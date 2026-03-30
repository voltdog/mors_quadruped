cmake_minimum_required(VERSION 3.28.3)
# set the project name
project(locomotionControllerMPC VERSION 1.0)

option(LOCOMOTION_ENABLE_IPO "Enable IPO/LTO on optimized builds" ON)
option(LOCOMOTION_ENABLE_NATIVE_OPT "Enable native CPU tuning and AVX flags" ON)
option(LOCOMOTION_ENABLE_EIGEN_NODEBUG "Disable Eigen runtime checks" ON)

get_filename_component(LCM_MSGS_DIR_DEFAULT "${CMAKE_CURRENT_LIST_DIR}/../lcm_msgs" ABSOLUTE)
if(NOT DEFINED LCM_MSGS_DIR OR NOT EXISTS "${LCM_MSGS_DIR}")
  set(LCM_MSGS_DIR "${LCM_MSGS_DIR_DEFAULT}" CACHE PATH "Path to lcm_msgs checkout" FORCE)
endif()
set(VBCONTROL_DIR "/home/$ENV{USER}/vbcontrol" CACHE PATH "Path to vbcontrol checkout")

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

# check AVX2 support

    set(AVX_FLAGS)

    include(CheckCXXSourceRuns)
    set(CMAKE_REQUIRED_FLAGS)

    # Check for AVX
    if (MSVC)
        if (NOT MSVC_VERSION LESS 1600)
            set(CMAKE_REQUIRED_FLAGS "/arch:AVX")
        endif ()
    else ()
        set(CMAKE_REQUIRED_FLAGS "-mavx")
    endif ()

    check_cxx_source_runs("
        #include <immintrin.h>
        int main()
        {
          __m256 a, b, c;
          const float src[8] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f };
          float dst[8];
          a = _mm256_loadu_ps( src );
          b = _mm256_loadu_ps( src );
          c = _mm256_add_ps( a, b );
          _mm256_storeu_ps( dst, c );
          for( int i = 0; i < 8; i++ ){
            if( ( src[i] + src[i] ) != dst[i] ){
              return -1;
            }
          }
          return 0;
        }"
            HAVE_AVX_EXTENSIONS)

    # Check for AVX2
    if (MSVC)
        if (NOT MSVC_VERSION LESS 1800)
            set(CMAKE_REQUIRED_FLAGS "/arch:AVX2")
        endif ()
    else ()
        set(CMAKE_REQUIRED_FLAGS "-mavx2")
    endif ()

    check_cxx_source_runs("
        #include <immintrin.h>
        int main()
        {
          __m256i a, b, c;
          const int src[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
          int dst[8];
          a =  _mm256_loadu_si256( (__m256i*)src );
          b =  _mm256_loadu_si256( (__m256i*)src );
          c = _mm256_add_epi32( a, b );
          _mm256_storeu_si256( (__m256i*)dst, c );
          for( int i = 0; i < 8; i++ ){
            if( ( src[i] + src[i] ) != dst[i] ){
              return -1;
            }
          }
          return 0;
        }"
            HAVE_AVX2_EXTENSIONS)

    # Set Flags according to check results
    if (MSVC)
        if (HAVE_AVX2_EXTENSIONS AND NOT MSVC_VERSION LESS 1800)
            list(APPEND AVX_FLAGS /arch:AVX2)
        elseif (HAVE_AVX_EXTENSIONS AND NOT MSVC_VERSION LESS 1600)
            list(APPEND AVX_FLAGS /arch:AVX)
        endif ()
    else ()
        if (HAVE_AVX2_EXTENSIONS)
            list(APPEND AVX_FLAGS -mavx2)
            message(STATUS "AVX2 is supported")
        elseif (HAVE_AVX_EXTENSIONS)
            list(APPEND AVX_FLAGS -mavx)
            message(STATUS "AVX is supported")
        else()
            message(WARNING "AVX is not supported")
        endif ()
    endif ()
#===============================================================================

# add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++20")

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(lcm REQUIRED)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(yaml-cpp CONFIG QUIET)

include(CheckIPOSupported)
include(FetchContent)

if(NOT yaml-cpp_FOUND)
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.8.0
  )
  FetchContent_MakeAvailable(yaml-cpp)
endif()

include(${LCM_USE_FILE})
include_directories("include/")
include_directories("../common/include/")
include_directories("${LCM_MSGS_DIR}/")
include_directories(include "${VBCONTROL_DIR}/include")

# Put all message definition files in the type directory in one list
FILE(GLOB message_definitions "${LCM_MSGS_DIR}/*.lcm")

# Generate headers from message definition
lcm_wrap_types(CPP_HEADERS cpp_headers
  ${message_definitions})

# Create library from all the messages
lcm_add_library(messages-cpp CPP ${cpp_headers})
target_include_directories(messages-cpp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

# executable
add_subdirectory(${VBCONTROL_DIR} ${PROJECT_BINARY_DIR}/build/vbcontrol)
target_link_libraries(vbcontrol PUBLIC Eigen3::Eigen)

set(EXEC_SOURCES  src/main.cpp 
                  src/ConvexMpc.cpp 
                  src/LcmDataExchange.cpp
                  src/WBICThread.cpp
                  src/SwingTrajectoryGenerator.cpp
                  src/SwingController.cpp
                  src/FootStepPlanner.cpp
                  src/ContactStateFSM.cpp
                  src/ReferenceGenerator.cpp
                  src/ConvexMpcThread.cpp
                  src/GaitTransition.cpp
                  src/wbic/wbic_control.cpp
                  src/wbic/tasks/wbc_task.cpp
                  src/wbic/tasks/body_ori_task.cpp
                  src/wbic/tasks/body_pos_task.cpp
                  src/wbic/tasks/tip_pos_task.cpp
                  ../common/src/Robot.cpp
                  ../common/src/system_functions.cpp
                  ../common/src/SimpleGaitScheduler.cpp
)
add_executable(${PROJECT_NAME} ${EXEC_SOURCES})

check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)

target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:RelWithDebInfo>:-O2 -g>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  EIGEN_DONT_PARALLELIZE
  # Keep Eigen ABI/layout compatible with prebuilt Pinocchio binaries when
  # native AVX flags are enabled for this target.
  EIGEN_MAX_ALIGN_BYTES=16
  EIGEN_MAX_STATIC_ALIGN_BYTES=16
  $<$<BOOL:${LOCOMOTION_ENABLE_EIGEN_NODEBUG}>:EIGEN_NO_DEBUG>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE COAL_DISABLE_HPP_FCL_WARNINGS)


if(LOCOMOTION_ENABLE_NATIVE_OPT)
  target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-march=native -mtune=native ${AVX_FLAGS}>
    $<$<CONFIG:RelWithDebInfo>:-march=native -mtune=native ${AVX_FLAGS}>
  )
endif()

if(LOCOMOTION_ENABLE_IPO AND ipo_supported)
  set_property(TARGET ${PROJECT_NAME} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
elseif(LOCOMOTION_ENABLE_IPO)
  message(STATUS "IPO/LTO is not supported: ${ipo_error}")
endif()

target_link_libraries(${PROJECT_NAME}  
                        OsqpEigen::OsqpEigen
                        vbcontrol
                        Eigen3::Eigen
                        yaml-cpp::yaml-cpp
                        pinocchio::pinocchio
                        pinocchio::pinocchio_parsers
                        eiquadprog::eiquadprog)

lcm_target_link_libraries(${PROJECT_NAME} messages-cpp ${LCM_NAMESPACE}lcm)
