cmake_minimum_required(VERSION 3.5)
project(baxter_gazebosim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  find_package(ignition-msgs8 REQUIRED)
  add_compile_definitions(-DIGNITION_GAZEBO)
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Fortress")
else()
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)
  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden")
endif()



set(CMAKE_CXX_STANDARD 17)

ament_auto_add_executable(baxter_gz_bridge src/bridge.cpp src/arm_node.cpp)
target_link_libraries(baxter_gz_bridge
                      ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
                      ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
                      )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_auto_package()
