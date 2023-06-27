cmake_minimum_required(VERSION 2.8.3)
project(me416_environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  message_generation
  apriltag_ros
  # gazebo_msgs
  me416_lab
)
# Depending on system install of Gazebo
find_package(gazebo REQUIRED)


catkin_python_setup()


# Add Messages
add_message_files(
  FILES
  Bearings.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
   # message_generation
   message_runtime
   apriltag_ros
   # gazebo_msgs
   me416_lab
)

catkin_install_python(
  PROGRAMS
  scripts/bearing_measurements_gazebo.py
  scripts/bearing_controller_gazebo.py
  scripts/motor_command.py
  # scripts/key_op.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo Plugins
#~ add_library(temp_plugin SHARED src/temp_plugin.cpp)
#~ target_link_libraries(temp_plugin ${GAZEBO_LIBRARIES})
#~ add_dependencies(temp_plugin ${catkin_EXPORTED_TARGETS})

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
