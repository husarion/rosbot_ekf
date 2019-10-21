cmake_minimum_required(VERSION 2.8.3)
project(rosbot_ekf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  Imu.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Configuration.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES rosbot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(msgs_conversion
  src/msgs_conversion.cpp
)
add_dependencies(msgs_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(msgs_conversion
  ${catkin_LIBRARIES}
)

add_executable(cmd_vel_ws2812b_example
  src/cmd_vel_ws2812b_example.cpp
)
add_dependencies(cmd_vel_ws2812b_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_ws2812b_example
  ${catkin_LIBRARIES}
)

add_executable(move_base_ws2812b_example
  src/move_base_ws2812b_example.cpp
)
add_dependencies(move_base_ws2812b_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_ws2812b_example
  ${catkin_LIBRARIES}
)
