cmake_minimum_required(VERSION 3.5)
project(baxter_gazebosim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(baxter_core_msgs REQUIRED)

find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_executable(baxter_gz_bridge src/bridge.cpp)
target_include_directories(baxter_gz_bridge PRIVATE include)
target_link_libraries(baxter_gz_bridge 
                      ignition-msgs${IGN_MSGS_VER}::core
                      ignition-transport${IGN_TRANSPORT_VER}::core)
ament_target_dependencies(baxter_gz_bridge rclcpp sensor_msgs baxter_core_msgs)

install(TARGETS baxter_gz_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_package()
