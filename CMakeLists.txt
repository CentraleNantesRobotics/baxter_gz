cmake_minimum_required(VERSION 3.5)
project(baxter_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Gazebo version, use env variable or ROS 2 official pairing for LTS
# https://gazebosim.org/docs/latest/ros_installation/
if(DEFINED ENV{GZ_VERSION})
  set(GZ_VERSION "$ENV{GZ_VERSION}")
elseif(DEFINED ENV{IGN_VERSION})
  set(GZ_VERSION "$ENV{IGN_VERSION}")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  set(GZ_VERSION "jetty")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
  set(GZ_VERSION "harmonic")
else()
  set(GZ_VERSION "fortress")
endif()

message(STATUS "Compiling against Gazebo ${GZ_VERSION}")

if("${GZ_VERSION}" STREQUAL "jetty")
  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS msgs12)
  set(GZ_TRANSPORT transport15)

elseif("${GZ_VERSION}" STREQUAL "harmonic")
  set(GZ_TARGET_PREFIX gz)
  set(GZ_TRANSPORT transport13)
  set(GZ_MSGS msgs10)

elseif("${GZ_VERSION}" STREQUAL "garden")
  set(GZ_TARGET_PREFIX gz)
  set(GZ_TRANSPORT transport12)
  set(GZ_MSGS msgs9)

else()
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_TRANSPORT transport11)
  set(GZ_MSGS msgs8)
  add_definitions(-DFOR_IGNITION_FORTRESS)
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
 
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
