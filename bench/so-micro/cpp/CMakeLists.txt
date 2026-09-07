cmake_minimum_required(VERSION 3.16)
project(franka_rs_so_micro_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This target does not link libfranka: franka::Model (0.9.2) can only be constructed from
# a franka::Network, i.e. by downloading the model library from a robot, which is exactly
# what an offline benchmark cannot do. main.cpp instead reproduces LibraryLoader,
# ModelLibrary and the Model:: member bodies verbatim from the 0.9.2 sources, so the only
# external dependency is the Poco::SharedLibrary that LibraryLoader is built on.
find_library(POCO_FOUNDATION_LIBRARY NAMES PocoFoundation REQUIRED)

add_executable(so_micro_cpp main.cpp)

# -O3, matching bench/cpp and the Rust side's opt-level = 3.
target_compile_options(so_micro_cpp PRIVATE -O3 -Wall -Wextra)

target_link_libraries(so_micro_cpp PRIVATE "${POCO_FOUNDATION_LIBRARY}" dl)
