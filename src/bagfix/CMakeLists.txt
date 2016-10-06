cmake_minimum_required(VERSION 2.8.3)
project(bagfix)

add_definitions(--std=c++11)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rosbag_storage
  rospy
  tf
  aruco
  sensor_msgs
  std_msgs
#  aruco_msgs
)

#generate_dynamic_reconfigure_options(
# cfg/ArucoThreshold.cfg
#)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
 FILES
 Plane.msg
 Marker2D.msg
 Markers2D.msg

 )
 generate_messages( DEPENDENCIES std_msgs
 )
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
)



link_libraries(${catkin_LIBRARIES})

add_executable(arucooff precisearuco.cpp)

add_executable(extractcamproj extractcamproj.cpp)

add_executable(fixarucodepth fixarucodepth.cpp)

add_executable(decompressdepth decompressdepth.cpp)

add_executable(arucoboard arucoboard.cpp)


#############
## Install ##
#############

install(TARGETS arucooff
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

#foreach(dir etc launch)
#    install(DIRECTORY ${dir}/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)

