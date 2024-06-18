cmake_minimum_required(VERSION 3.5)
project(baxter_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR "$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
  set(GZ_TARGET_PREFIX gz)

  set(GZ_TRANSPORT transport13)
  set(GZ_MSGS msgs10)

  message(STATUS "Compiling against Gazebo Harmonic")

# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  set(GZ_TARGET_PREFIX gz)

  set(GZ_TRANSPORT transport12)
  set(GZ_MSGS msgs9)

  message(STATUS "Compiling against Gazebo Garden")

# Default to Fortress
else()
  set(GZ_TARGET_PREFIX ignition)

  set(GZ_TRANSPORT transport11)
  set(GZ_MSGS msgs8)

  add_definitions(-DFOR_IGNITION_FORTRESS)
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(${GZ_TARGET_PREFIX}-${GZ_TRANSPORT})
find_package(${GZ_TARGET_PREFIX}-${GZ_MSGS})

set(CMAKE_CXX_STANDARD 17)

ament_auto_add_executable(baxter_gz_bridge src/bridge.cpp src/arm_node.cpp)
target_include_directories(baxter_gz_bridge PRIVATE
                          ${GZ_TARGET_PREFIX}/${GZ_TRANSPORT}
                          ${GZ_TARGET_PREFIX}/${GZ_MSGS})
target_link_libraries(baxter_gz_bridge
                      ${GZ_TARGET_PREFIX}-${GZ_TRANSPORT}::core
                      ${GZ_TARGET_PREFIX}-${GZ_MSGS}::core
                      )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
 
ament_auto_package()
