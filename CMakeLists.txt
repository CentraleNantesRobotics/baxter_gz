cmake_minimum_required(VERSION 3.5)
project(baxter_gazebosim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

set(CMAKE_CXX_STANDARD 17)

ament_auto_add_executable(cmd_bridge src/bridge.cpp src/arm_node.cpp)
target_link_libraries(cmd_bridge 
                      ignition-msgs${IGN_MSGS_VER}::core
                      ignition-transport${IGN_TRANSPORT_VER}::core)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_auto_package()
