cmake_minimum_required(VERSION 3.16)
project(franka_rs_bench_cpp CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Point these at the prebuilt libfranka tree that the benchmark links against (libfranka
# 0.20.4, built with RealtimeConfig defaults left alone; see bench/README.md). Override with
# -DLIBFRANKA_SOURCE_DIR=/path/to/libfranka if it lives somewhere other than $HOME/libfranka.
set(LIBFRANKA_SOURCE_DIR "$ENV{HOME}/libfranka"
    CACHE PATH "libfranka source tree (for headers and examples_common)")
set(LIBFRANKA_BUILD_DIR "${LIBFRANKA_SOURCE_DIR}/build-rt-ignore"
    CACHE PATH "libfranka build tree (for libfranka.so and libexamples_common.a)")

find_path(EIGEN3_INCLUDE_DIR signature_of_eigen3_matrix_library
          HINTS /usr/include/eigen3 /usr/local/include/eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 headers not found")
endif()

find_library(LIBFRANKA_LIBRARY
             NAMES franka
             HINTS "${LIBFRANKA_BUILD_DIR}"
             NO_DEFAULT_PATH)
if(NOT LIBFRANKA_LIBRARY)
  message(FATAL_ERROR "libfranka.so not found in ${LIBFRANKA_BUILD_DIR}")
endif()

find_library(EXAMPLES_COMMON_LIBRARY
             NAMES examples_common
             HINTS "${LIBFRANKA_BUILD_DIR}/examples"
             NO_DEFAULT_PATH)
if(NOT EXAMPLES_COMMON_LIBRARY)
  message(FATAL_ERROR "libexamples_common.a not found in ${LIBFRANKA_BUILD_DIR}/examples; "
                      "build the libfranka examples first")
endif()

# The version of the libfranka being linked, so each run's JSON records the library it
# actually ran against instead of a hard-coded string. Read off the versioned soname.
set(LIBFRANKA_VERSION "unknown")
file(GLOB _libfranka_sonames "${LIBFRANKA_BUILD_DIR}/libfranka.so.*")
foreach(_soname IN LISTS _libfranka_sonames)
  if(_soname MATCHES "libfranka\\.so\\.([0-9]+\\.[0-9]+\\.[0-9]+)$")
    set(LIBFRANKA_VERSION "${CMAKE_MATCH_1}")
  endif()
endforeach()
message(STATUS "linking libfranka ${LIBFRANKA_VERSION} from ${LIBFRANKA_BUILD_DIR}")

add_executable(bench_joint_velocity bench_joint_velocity.cpp)
target_include_directories(bench_joint_velocity PRIVATE
  "${LIBFRANKA_SOURCE_DIR}/include"
  "${LIBFRANKA_SOURCE_DIR}/common/include"
  "${LIBFRANKA_SOURCE_DIR}/examples"
  "${EIGEN3_INCLUDE_DIR}")
target_compile_options(bench_joint_velocity PRIVATE -O3 -Wall -Wextra)
target_link_libraries(bench_joint_velocity PRIVATE
  "${EXAMPLES_COMMON_LIBRARY}" "${LIBFRANKA_LIBRARY}")
set_target_properties(bench_joint_velocity PROPERTIES
  BUILD_RPATH "${LIBFRANKA_BUILD_DIR}"
  INSTALL_RPATH "${LIBFRANKA_BUILD_DIR}")

add_executable(bench_model_control bench_model_control.cpp)
target_include_directories(bench_model_control PRIVATE
  "${LIBFRANKA_SOURCE_DIR}/include"
  "${LIBFRANKA_SOURCE_DIR}/common/include"
  "${LIBFRANKA_SOURCE_DIR}/examples"
  "${EIGEN3_INCLUDE_DIR}")
target_compile_options(bench_model_control PRIVATE -O3 -Wall -Wextra)
target_compile_definitions(bench_model_control PRIVATE
  BENCH_LIBFRANKA_VERSION="${LIBFRANKA_VERSION}")
target_link_libraries(bench_model_control PRIVATE
  "${EXAMPLES_COMMON_LIBRARY}" "${LIBFRANKA_LIBRARY}")
set_target_properties(bench_model_control PROPERTIES
  BUILD_RPATH "${LIBFRANKA_BUILD_DIR}"
  INSTALL_RPATH "${LIBFRANKA_BUILD_DIR}")

add_executable(robot_home robot_home.cpp)
target_include_directories(robot_home PRIVATE
  "${LIBFRANKA_SOURCE_DIR}/include"
  "${LIBFRANKA_SOURCE_DIR}/common/include"
  "${LIBFRANKA_SOURCE_DIR}/examples"
  "${EIGEN3_INCLUDE_DIR}")
target_compile_options(robot_home PRIVATE -O3 -Wall -Wextra)
target_link_libraries(robot_home PRIVATE
  "${EXAMPLES_COMMON_LIBRARY}" "${LIBFRANKA_LIBRARY}")
set_target_properties(robot_home PROPERTIES
  BUILD_RPATH "${LIBFRANKA_BUILD_DIR}"
  INSTALL_RPATH "${LIBFRANKA_BUILD_DIR}")

# A portable CPU hog for the "loaded" condition: this box has no stress-ng.
add_executable(cpu_hog cpu_hog.c)
target_compile_options(cpu_hog PRIVATE -O2)
target_link_libraries(cpu_hog PRIVATE m)
