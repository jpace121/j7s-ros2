cmake_minimum_required(VERSION 3.5)
project(blinkt_test)

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
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(blinkt_interface REQUIRED)

add_executable(rainbow src/rainbow.cpp)
target_include_directories(rainbow PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rainbow blinkt_interface::blinkt)
ament_target_dependencies(rainbow
  "blinkt_interface"
  "rclcpp"
)

add_executable(brightness src/brightness.cpp)
target_include_directories(brightness PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(brightness blinkt_interface::blinkt)
ament_target_dependencies(brightness
  "blinkt_interface"
  "rclcpp"
)
# Install targets.
install(TARGETS rainbow brightness
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
