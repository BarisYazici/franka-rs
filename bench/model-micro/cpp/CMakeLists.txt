cmake_minimum_required(VERSION 3.16)
project(franka_rs_model_micro_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Root of the libfranka 0.20.4 source tree whose build directory holds libfranka.so.
# Same defaults as tools/model-reference and bench/cpp.
set(LIBFRANKA_DIR "$ENV{HOME}/libfranka"
    CACHE PATH "libfranka 0.20.4 source root")
set(LIBFRANKA_BUILD_DIR "${LIBFRANKA_DIR}/build-rt-ignore"
    CACHE PATH "libfranka 0.20.4 build directory containing libfranka.so")

# Pinocchio / Eigen include paths, taken verbatim from tools/model-reference/CMakeLists.txt:
# <franka/robot_model.h> pulls the Pinocchio headers in directly.
set(EIGEN_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Eigen 3 include directory")
set(PINOCCHIO_PREFIX "/opt/openrobots" CACHE PATH "Pinocchio installation prefix")

find_library(LIBFRANKA_LIBRARY
    NAMES franka
    PATHS "${LIBFRANKA_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED)

add_executable(model_micro_cpp main.cpp)

target_include_directories(model_micro_cpp PRIVATE
    "${LIBFRANKA_DIR}/include"
    "${LIBFRANKA_DIR}/common/include")

target_include_directories(model_micro_cpp SYSTEM PRIVATE
    "${EIGEN_INCLUDE_DIR}"
    "${PINOCCHIO_PREFIX}/include"
    "${PINOCCHIO_PREFIX}/include/pinocchio/deprecated")

target_compile_definitions(model_micro_cpp PRIVATE
    BOOST_FUSION_INVOKE_MAX_ARITY=12
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_URDFDOM_HEADERS_MAJOR_VERSION=1
    PINOCCHIO_URDFDOM_HEADERS_MINOR_VERSION=0
    PINOCCHIO_URDFDOM_HEADERS_PATCH_VERSION=5)

# -O3, matching bench/cpp and the Rust side's opt-level = 3.
target_compile_options(model_micro_cpp PRIVATE -O3 -Wall -Wextra)

target_link_libraries(model_micro_cpp PRIVATE "${LIBFRANKA_LIBRARY}")
target_link_directories(model_micro_cpp PRIVATE "${PINOCCHIO_PREFIX}/lib")
set_target_properties(model_micro_cpp PROPERTIES
    BUILD_RPATH "${LIBFRANKA_BUILD_DIR};${PINOCCHIO_PREFIX}/lib")
