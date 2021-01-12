cmake_minimum_required(VERSION 3.0.2)
project(maze_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS roscpp message_generation std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/lib)

## Generate messages in the 'msg' folder
add_message_files(FILES
  MazeRobotState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp
#  DEPENDS system_lib
)

add_executable(motor_vel_pub src/motor_control/Encoder/motor_vel_pub.cpp)

target_link_libraries(motor_vel_pub ${catkin_LIBRARIES} -lwiringPi -lm -lpthread -lcrypt -lrt)

add_dependencies(motor_vel_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/keyboard_control/teleop_twist_keyboard
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
