cmake_minimum_required(VERSION 2.8.3)
project(vision_batch)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(noos REQUIRED)
if (NOT NOOS_LIBRARIES)
    message(FATAL_ERROR "No NOOS library found")
endif()
message(STATUS "${NOOS_LIBRARIES}")
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
                CATKIN_DEPENDS roscpp std_msgs 
              )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
                      ${catkin_INCLUDE_DIRS}
                      ${NOOS_INCLUDE_DIRS}
                      ${OpenCV_INCLUDE_DIRS}
                   )

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/vision.cpp
              )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=gnu++1y -Wall -mtune=native -Wno-write-strings -Wno-narrowing")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node 
                        ${catkin_LIBRARIES}
						${OPENSSL_LIBRARIES} 
						${Boost_LIBRARIES}
						${CMAKE_THREAD_LIBS_INIT}
						${NOOS_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                     )
