cmake_minimum_required(VERSION 2.8.3)
project(bluefox)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge

  sensor_msgs
  std_msgs
  message_filters
  image_transport
)
catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS 

)

set(mvIMPACT_DIR /opt/mvIMPACT_Acquire)
set(mvIMPACT_LIBRARIES_DIR /opt/mvIMPACT_Acquire/lib/x86_64)

link_directories(
	${mvIMPACT_LIBRARIES_DIR}
)
link_libraries(
	mvDeviceManager 
	mvImageProcessing 
	mvIMPACT_Acquire.java
	mvPropHandling  
	usb-1.0
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  ${mvIMPACT_DIR}
 )

set(SRC ${SRC} src/main.cpp)

set(DCMAKE_build_type release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")
add_executable (bluefox_multiple_node ${SRC} )
add_dependencies(bluefox_multiple_node ${catkin_EXPORTED_TARGETS})
target_link_libraries (bluefox_multiple_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} )
