cmake_minimum_required(VERSION 3.0)

set(PROJECT_NAME fcu_simulator)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE OFF)

find_package(Gazebo REQUIRED)
message("Gazebo include: ${GAZEBO_INCLUDE_DIRS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(/usr/include/gazebo-9/gazebo/msgs/)
include_directories(/home/kugua/workspace/PX4_Firmware/PX4-Autopilot/build/px4_sitl_default/build_gazebo/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3)
if(EIGEN3_FOUND)
    message("Eigen3 include: ${EIGEN3_INCLUDE_DIR}")
    include_directories(${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

include_directories(/home/kugua/workspace/PX4_Firmware/PX4-Autopilot/mavlink/include/mavlink/v2.0/)
include_directories(/home/kugua/workspace/catkin_ws/devel/include/)

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} 
  ${GAZEBO_LIBRARIES} 
  ${catkin_LIBRARIES} 
  /home/kugua/workspace/PX4_Firmware/PX4-Autopilot/build/px4_sitl_default/build_gazebo/libmav_msgs.so
  /home/kugua/workspace/PX4_Firmware/PX4-Autopilot/build/px4_sitl_default/build_gazebo/libsensor_msgs.so
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-address-of-packed-member -Wno-cast-align)
