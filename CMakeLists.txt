cmake_minimum_required(VERSION 2.4.6)
cmake_policy(SET CMP0003 OLD)

set(PROJECT_NAME camthing)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build )
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build )

include_directories( . )
#include_directories( $ENV{HG_CLONE_DIR}/external/include )
include_directories( $ENV{HOME}/other/install/include )


include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")

###############################################################################
if (false)
set(CMAKE_FIND_LIBRARY_SUFFICES ".a;.so")
find_library(OPEN_CV2 opencv_core opencv_imgproc opencv_highgui opencv_calib3d $ENV{HOME}/other/install/lib )

if (NOT OPEN_CV2)
  MESSAGE("OpenCV not found")
  endif (NOT OPEN_CV2)

MESSAGE(${OPEN_CV2})

set(CV_LIBS 
opencv_imgproc 
opencv_highgui 
opencv_features2d
opencv_flann
)

set(SOME_LIBS ${OPEN_CV2}  
${CV_LIBS}
 )
endif()

find_package(OpenCV)
###############################################################################

add_executable(${PROJECT_NAME}
camthing.cpp
nodes.cpp
misc_nodes.cpp
signals.cpp
filter.cpp
generate.cpp
screencap.cpp
)

target_link_libraries(${PROJECT_NAME} 
#${SOME_LIBS} 
${OpenCV_LIBS}
glog
gflags
boost_thread
boost_filesystem
boost_system
)


