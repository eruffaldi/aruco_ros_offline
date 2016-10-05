cmake_minimum_required(VERSION 2.8.3)
project(arucooff)

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
#  aruco_msgs
)

#generate_dynamic_reconfigure_options(
# cfg/ArucoThreshold.cfg
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arucooff precisearuco.cpp)

add_executable(fixarucodepth fixarucodepth.cpp)


#add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(arucooff ${catkin_LIBRARIES})


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

