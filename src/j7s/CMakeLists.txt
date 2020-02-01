cmake_minimum_required(VERSION 3.5)
project(j7s)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(blinkt_interface REQUIRED)
find_package(j7s_msgs REQUIRED)

add_executable(j7s-sub src/j7s-sub.cpp)
target_include_directories(j7s-sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  j7s-sub
  "rclcpp"
  "blinkt_interface"
  "j7s_msgs"
)

add_executable(j7s-pub src/j7s-pub.cpp)
target_include_directories(j7s-pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  j7s-pub
  "rclcpp"
  "j7s_msgs"
)

install(TARGETS j7s-sub j7s-pub
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
