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
set(NOOS_INCLUDE "/home/ort/noos-api-cpp/")
find_library(NOOS_LIBRARY NAMES noos REQUIRED PATHS "/home/ort/noos-api-cpp/build/")
message(STATUS "${NOOS_LIBRARY}")
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(Threads REQUIRED)

set(NOOS_LIBRARIES ${NOOS_LIBRARY} 
                   ${OPENSSL_LIBRARIES} 
                   ${Boost_LIBRARIES}
                   ${CMAKE_THREAD_LIBS_INIT})

include_directories(${catkin_INCLUDE_DIRS} 
                    ${NOOS_INCLUDE}
                   )

add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/slam.cpp
		      )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-std=gnu++1y -Wall -s -O3 -mtune=native -Wno-write-strings -Wno-narrowing")
target_link_libraries(${PROJECT_NAME}_node 
                      ${catkin_LIBRARIES}
                      ${NOOS_LIBRARIES}
                      )
