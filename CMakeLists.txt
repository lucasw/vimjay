cmake_minimum_required(VERSION 3.0.2)
project(vimjay)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/BackgroundSubtraction.cfg
  cfg/DrCameraInfo.cfg
  cfg/ImageDeque.cfg
)

catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
  roscpp
  rospy
  sensor_msgs
  std_msgs
  DEPENDS OpenCV
)

set(ROSLINT_PYTHON_OPTS "--ignore=''" "1>&2")
roslint_python()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  other/dso_noises/cellular.c
  other/dso_noises/noise1234.c
  other/dso_noises/sdnoise1234.c
  other/dso_noises/simplexnoise1234.c
  other/dso_noises/srdnoise23.c
  other/simplexnoise.cpp
  other/simplextextures.cpp
  src/cluster.cpp
  src/config.cpp
  src/filter.cpp
  src/generate.cpp
  src/image_dir.cpp
  src/misc_nodes.cpp
  src/modify.cpp
  src/nodes.cpp
  src/output.cpp
  src/signals.cpp
  src/structure.cpp
  src/utility.cpp
  src/video.cpp
  src/vimjay.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  boost_thread
  boost_filesystem
  boost_system
  boost_timer
  ${OpenCV_LIBRARIES}
)

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
  src/${PROJECT_NAME}.cpp
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
  ${OpenCV_LIBRARIES}
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

if (true)
  # Melodic in 18.04 needs this
  # find_package(OpenCV REQUIRED)

  add_executable(nn_im
    src/nngen/nn_im.cpp
  )
  target_link_libraries(nn_im
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
  )
endif()

catkin_install_python(PROGRAMS
  scripts/camera_info_to_plane.py
  scripts/display.py
  scripts/dr_camera_info.py
  scripts/image.py
  scripts/key_input.py
  scripts/subdiv.py
  scripts/test_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/grep_msgs.sh
  # TODO(lucasw) maybe this shouldn't be in scripts
  scripts/webcam.html
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  background_subtraction
  cam_sub
  color
  cv_distort_image
  distort_image
  iir_imagev
  image_deque
  key_trigger
  nn_im
  noise_image
  save_image
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
