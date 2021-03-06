cmake_minimum_required(VERSION 2.8.3)
project(slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
  sensor_msgs
)

catkin_package(
               CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs
              )

find_package(noos REQUIRED)
if (NOT NOOS_LIBRARIES)
    message(FATAL_ERROR "No NOOS library found")
endif()
message(STATUS "${NOOS_LIBRARIES}")
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Threads REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} 
                    ${NOOS_INCLUDE_DIRS}
                   )

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/slam.cpp
		      )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=gnu++1y -Wall -s -O3 -mtune=native -Wno-write-strings -Wno-narrowing")
target_link_libraries(${PROJECT_NAME}_node 
                      ${catkin_LIBRARIES}
                      ${NOOS_LIBRARIES}
                      ${OPENSSL_LIBRARIES} 
                      ${Boost_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT}
                      )
