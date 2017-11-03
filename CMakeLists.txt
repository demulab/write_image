cmake_minimum_required(VERSION 2.8.3)
project(write_image)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs joy cv_bridge image_transport)
## find_package(PCL 1.3 REQUIRED COMPONENTS common io )

#find_package(OpenCV REQUIRED)

## add demu
include_directories(/usr/local/include ~/catkin_ws/devel/include ${catkin_INCLUDE_DIRS} )#${OpenCV_INCLUDE_DIRS}  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES write_image
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS sensor_msgs roscpp
  )

###########
## Build ##
###########

add_executable(write_image src/write_image.cpp)
target_link_libraries(write_image ${catkin_LIBRARIES} )#${OpenCV_LIBRARIES} )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS write_image
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
