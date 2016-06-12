cmake_minimum_required(VERSION 2.8.3)
project(people_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport
  laser_geometry
  image_geometry
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED) 
find_package( PkgConfig REQUIRED) 
pkg_check_modules(gsl REQUIRED gsl)
pkg_check_modules(EIGEN REQUIRED eigen3)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gsl_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/
  src/legs_detector/LegsDetector/
  src/legs_detector/features/
  src/legs_detector/common/
  src/legs_detector/linalg/
)

aux_source_directory(src/legs_detector/LegsDetector src1)
aux_source_directory(src/legs_detector/common src2)
aux_source_directory(src/legs_detector/features src3)
aux_source_directory(src/legs_detector/linalg src4)

## Declare a cpp executable
add_executable(people_detector_node 	src/PeopleDetector.cpp 
					src/LaserDetector.cpp 
					src/ImageDetector.cpp 
					src/Person.cpp
					src/PeopleDetectorNode.cpp
					${src1}
					${src2}
					${src3}
					${src4}
				)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(people_detector_node $catkin_EXPORTED_TARGETS)

## Specify libraries to link a library or executable target against
 target_link_libraries(people_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${gsl_LIBRARIES}
   ${EIGEN_LIBRARIES}
 )

add_definitions(${EIGEN_DEFINITIONS})
