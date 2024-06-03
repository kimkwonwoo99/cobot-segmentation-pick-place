cmake_minimum_required(VERSION 3.0.2)
project(vision_seg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
  tf
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  aruco_center.msg
  seg_center.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  find_tf_service.srv
  SegImageService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS roscpp rospy tf sensor_msgs message_generation cv_bridge image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${Eigen_INCLUDE_DIRS})

add_executable(arucoTfPub src/arucoTfPub.cpp)
target_link_libraries(arucoTfPub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen3_LIBRARIES})

catkin_install_python(PROGRAMS src/seg_pose.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(cameraPub src/cameraPub.cpp)
target_link_libraries(cameraPub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(findNearAruco src/findNearAruco.cpp)
target_link_libraries(findNearAruco ${catkin_LIBRARIES})
