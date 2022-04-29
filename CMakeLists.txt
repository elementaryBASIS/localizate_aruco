cmake_minimum_required(VERSION 3.0.2)
project(localizate_aruco)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  
  roscpp
  rospy
  sensor_msgs
  std_msgs
  image_transport
)
catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS
  opencv
)
find_package(OpenCV)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(streamer_node src/streamer.cpp)
add_dependencies(streamer_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(streamer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
catkin_install_python(PROGRAMS
  python_scripts/staticMarkers.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
