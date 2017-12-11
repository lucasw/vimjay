cmake_minimum_required(VERSION 2.8.3)
project(vimjay)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  camera_info_manager
  dynamic_reconfigure
  image_transport
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
# this may be a bad idea given linkage of ROS stuff
# http://answers.ros.org/question/152276/is-there-a-way-to-enable-c11-support-for-catkin-packages/
# if this is eliminated get  error: ‘dynamic_pointer_cast’ was not declared in this scope

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

generate_dynamic_reconfigure_options(
  cfg/BackgroundSubtraction.cfg
  cfg/DrCameraInfo.cfg
  cfg/ImageDeque.cfg
)

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
  INCLUDE_DIRS include
#  LIBRARIES vimjay
  CATKIN_DEPENDS roscpp rospy std_msgs 
#  DEPENDS glog gflags
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  include/other
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(vimjay
#   src/${PROJECT_NAME}/vimjay.cpp
# )

## Declare a cpp executable
add_executable(vimjay_node 
other/DSOnoises/cellular.c
other/DSOnoises/noise1234.c
other/DSOnoises/sdnoise1234.c
other/DSOnoises/simplexnoise1234.c
other/DSOnoises/srdnoise23.c
src/config.cpp
src/vimjay.cpp
src/nodes.cpp
src/misc_nodes.cpp
src/image_dir.cpp
src/modify.cpp
src/signals.cpp
src/filter.cpp
src/cluster.cpp
src/generate.cpp
src/structure.cpp
src/utility.cpp
src/output.cpp
src/video.cpp
other/simplexnoise.cpp
other/simplextextures.cpp
)

# This is handled by external screencap node
#src/screencap.cpp
# TBD update to use ros joystick node, and TBD mouse input node
#src/input.cpp

#src/opengl.cpp


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
target_link_libraries(vimjay_node
   ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
boost_thread
boost_filesystem
boost_system

)

#Xi
#X11
#glut
#GLEW
#GL

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  src/nngen
  ${EIGEN3_INCLUDE_DIR}
)

roslint_cpp(
  src/standalone/noise.cpp
  src/standalone/background_subtraction.cpp
  src/standalone/color.cpp
  src/standalone/save_image.cpp
  src/standalone/key_trigger.cpp
  src/standalone/image_deque.cpp
  src/standalone/iir_image.cpp
  src/nngen/nn_im.cpp
  src/vimjay.cpp
  src/input.cpp
)

add_executable(noise_image
  src/standalone/noise.cpp
)
target_link_libraries(noise_image
  ${catkin_LIBRARIES}
)

add_executable(color
  src/standalone/color.cpp
)
target_link_libraries(color
  ${catkin_LIBRARIES}
)

add_executable(save_image
  src/standalone/save_image.cpp
)
target_link_libraries(save_image
  ${catkin_LIBRARIES}
)

add_executable(key_trigger
  src/standalone/key_trigger.cpp
)
target_link_libraries(key_trigger
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_deque
  src/standalone/image_deque.cpp
)
add_dependencies(image_deque ${PROJECT_NAME}_gencfg)
target_link_libraries(image_deque
  ${catkin_LIBRARIES}
)

add_executable(background_subtraction
  src/standalone/background_subtraction.cpp
)
add_dependencies(background_subtraction ${PROJECT_NAME}_gencfg)
target_link_libraries(background_subtraction
  ${catkin_LIBRARIES}
)

add_executable(cam_sub
  src/standalone/cam_sub.cpp
)
target_link_libraries(cam_sub
  ${catkin_LIBRARIES}
)
add_executable(iir_imagev
  src/standalone/iir_image.cpp
)
target_link_libraries(iir_imagev
  ${catkin_LIBRARIES}
)

add_library(cv_distort_image
  src/standalone/cv_distort_image.cpp
)

add_executable(distort_image
  src/standalone/distort_image.cpp
)
target_link_libraries(distort_image
  cv_distort_image
  ${catkin_LIBRARIES}
)

if (false)
add_executable(nn_im
  src/nngen/nn_im.cpp
)
target_link_libraries(nn_im
  ${catkin_LIBRARIES}
)
endif()


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
install(TARGETS vimjay_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
catkin_add_gtest(image_distort_test test/image_distort_test.cpp)
if(TARGET image_distort_test)
  target_link_libraries(image_distort_test ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
