cmake_minimum_required(VERSION 2.8.3)
project(multiple_goals)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf
  
)

 ##Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES
 #   DriveCommands.msg
 #   DriveStates.msg
 #   EmergencyStopState.msg
 #   GyroBoard.msg
#	IOAnalogIn.msg
#	IOOut.msg
#	IRSensors.msg
#	Keypad.msg
#	LCDOutput.msg
#	RadarBoard.msg
#	Temperatur.msg
#	USBoard.msg
 #)

## Generate services in the 'srv' folder
## add_service_files(
##   FILES
##   Service1.srv
##   Service2.srv
## )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

 ##Generate added messages and services with any dependencies listed here
 #generate_messages(
 #  DEPENDENCIES
 #  std_msgs
 #)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_relayboard
# CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(multiple_goals ros/src/multiple_goals_node.cpp)
##target_link_libraries(multiple_goals ${PROJECT_NAME})
target_link_libraries(multiple_goals ${catkin_LIBRARIES})
add_dependencies(multiple_goals ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS neo_relayboard neo_relayboard_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_neo_relayboard.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
