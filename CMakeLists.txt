cmake_minimum_required(VERSION 2.8.3)
project(camera_response_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  image_transport
  cv_bridge
  nodelet
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_response_test
  CATKIN_DEPENDS roscpp nodelet std_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/camera_response_test_nodelet.cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(${PROJECT_NAME}
  pigpio
  pthread
  rt
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(camera_response_test ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS camera_response_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES camera_response_test.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
