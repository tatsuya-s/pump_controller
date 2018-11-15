cmake_minimum_required(VERSION 2.8.3)
project(pump_controller)

include(pigpio/util/Findpigpio.cmake)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_srvs
    message_generation
    actionlib
    actionlib_msgs
)

find_package(Threads REQUIRED)
find_package(pigpio)
if(NOT pigpio_FOUND)
  add_subdirectory(pigpio)
  set(pigpio_INCLUDE_DIR pigpio)
  set(pigpiod_if2_LIBRARY pigpiod_if2)
endif()

add_service_files(
  FILES
  ControlPump.srv
)

add_action_files(
  FILES
  ControlPump.action
)

generate_messages(
  DEPENDENCIES
    std_srvs
    actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    pump_controller
  CATKIN_DEPENDS
    roscpp
    rospy
    std_srvs
    message_runtime
    actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pigpio_INCLUDE_DIR}
)

add_executable(pump_controller_node src/pump_controller_node.cpp)
target_link_libraries(pump_controller_node ${catkin_LIBRARIES} ${pigpiod_if2_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(pump_controller_node pump_controller_generate_messages_cpp)