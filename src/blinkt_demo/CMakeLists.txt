cmake_minimum_required(VERSION 3.5)
project(blinkt_demo)

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
find_package(blinkt_interface REQUIRED)

add_executable(show_one src/show_one.cpp)
target_link_libraries(show_one blinkt_interface::blinkt)
ament_target_dependencies(show_one
  "blinkt_interface"
)

add_executable(all_off src/all_off.cpp)
target_link_libraries(all_off blinkt_interface::blinkt)
ament_target_dependencies(all_off
  "blinkt_interface"
)

add_executable(run src/run.cpp)
target_link_libraries(run blinkt_interface::blinkt)
ament_target_dependencies(run
  "blinkt_interface"
)

add_executable(rainbow src/rainbow.cpp)
target_link_libraries(rainbow blinkt_interface::blinkt)
ament_target_dependencies(rainbow
  "blinkt_interface"
)

add_executable(brightness src/brightness.cpp)
target_link_libraries(brightness blinkt_interface::blinkt)
ament_target_dependencies(brightness
  "blinkt_interface"
)

# Install targets.
install(TARGETS rainbow brightness show_one all_off run
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
