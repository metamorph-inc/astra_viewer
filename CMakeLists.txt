cmake_minimum_required(VERSION 2.8.3)
project(astra_viewer)
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

## Find Astra and PCL packages
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Astra REQUIRED)
find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package()

####################
## Build
####################
set (_projname astra_viewer)

## declare project sources
set (${_projname}_SOURCES
  src/main.cpp
  src/publisher.cpp
  src/publisher.h
  src/frameListener.h
  src/key_handler.h
  src/common.h
)

## declare include directories
include_directories(
	${catkin_INCLUDE_DIRS}
	${ASTRA_INCLUDE_DIR} 
	${PCL_INCLUDE_DIRS}
)

## declare C++ executable
add_executable(${_projname} ${${_projname}_SOURCES})

## specifie libraries to link
target_link_libraries(${_projname} 
	${catkin_LIBRARIES}
	${ASTRA_LIBRARIES} 
	${PCL_LIBRARY_DIRS}
)

