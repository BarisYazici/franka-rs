cmake_minimum_required(VERSION 3.16)
project(franka_rs_model_reference CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Root of the libfranka 0.20.4 source tree whose build directory holds
# libfranka.so.0.20.4. Override with -DLIBFRANKA_DIR=... if it moved.
set(LIBFRANKA_DIR "$ENV{HOME}/libfranka"
    CACHE PATH "libfranka 0.20.4 source root")
set(LIBFRANKA_BUILD_DIR "${LIBFRANKA_DIR}/build-rt-ignore"
    CACHE PATH "libfranka 0.20.4 build directory containing libfranka.so")

# Pinocchio 4.1 / Eigen include paths, taken verbatim from
# ${LIBFRANKA_BUILD_DIR}/compile_commands.json (the entry for src/robot_model.cpp).
set(EIGEN_INCLUDE_DIR "/usr/include/eigen3" CACHE PATH "Eigen 3 include directory")
set(PINOCCHIO_PREFIX "/opt/openrobots" CACHE PATH "Pinocchio installation prefix")

find_library(LIBFRANKA_LIBRARY
    NAMES franka
    PATHS "${LIBFRANKA_BUILD_DIR}"
    NO_DEFAULT_PATH
    REQUIRED)

add_executable(model-reference main.cpp)

target_include_directories(model-reference PRIVATE
    "${LIBFRANKA_DIR}/include"
    "${LIBFRANKA_DIR}/common/include")

target_include_directories(model-reference SYSTEM PRIVATE
    "${EIGEN_INCLUDE_DIR}"
    "${PINOCCHIO_PREFIX}/include"
    "${PINOCCHIO_PREFIX}/include/pinocchio/deprecated")

# The same Boost/Pinocchio configuration macros libfranka itself is compiled
# with; the Pinocchio headers pulled in by <franka/robot_model.h> require them.
target_compile_definitions(model-reference PRIVATE
    BOOST_FUSION_INVOKE_MAX_ARITY=12
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    PINOCCHIO_WITH_URDFDOM
    PINOCCHIO_URDFDOM_HEADERS_MAJOR_VERSION=1
    PINOCCHIO_URDFDOM_HEADERS_MINOR_VERSION=0
    PINOCCHIO_URDFDOM_HEADERS_PATCH_VERSION=5)

target_compile_options(model-reference PRIVATE -O2 -Wall -Wextra)

target_link_libraries(model-reference PRIVATE "${LIBFRANKA_LIBRARY}")
target_link_directories(model-reference PRIVATE "${PINOCCHIO_PREFIX}/lib")
set_target_properties(model-reference PROPERTIES
    BUILD_RPATH "${LIBFRANKA_BUILD_DIR};${PINOCCHIO_PREFIX}/lib")
