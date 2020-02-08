cmake_minimum_required(VERSION 3.5)
project(blinkt_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wno-psabi)
  endif()
endif()

# find dependencies
find_package(pybind11 REQUIRED)
find_package(blinkt_interface REQUIRED)

pybind11_add_module(blinkt_interface src/blinkt_interface.cpp)
target_link_libraries(blinkt_interface PUBLIC blinkt_interface::blinkt)
