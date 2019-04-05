cmake_minimum_required(VERSION 2.8.3)
project(rtklibros)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++11)
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  dynamic_reconfigure
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES hdl_scan_matching_odometry
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories ("${PROJECT_SOURCE_DIR}/src")

#pos2kml
add_library(convkml src/convkml.c)
add_library(solution src/solution.c)
add_library(geoid src/geoid.c)
add_library(rtkcmn src/rtkcmn.c)



#str2str
#add_library(ss2 src/rcv/ss2.c)
#add_library(crescent src/rcv/crescent.c)
#add_library(skytraq src/rcv/skytraq.c)
#add_library(gw10 src/rcv/gw10.c)
#add_library(javad src/rcv/javad.c)

add_library(stream src/stream.c)
add_library(sbas src/sbas.c)

#add_library(ublox src/rcv/ublox.c src/rcvraw.c)
#target_link_libraries(ublox ss2 crescent gw10 javad skytraq crescent sbas novatel rtkcmn rinex rtcm)
target_link_libraries(       sbas  rtkcmn rinex rtcm rtcm2 rtcm3 rtcm3e)

#add_library(novatel src/rcv/novatel.c)
add_library(streamsvr src/streamsvr.c)



add_library(rinex  src/rinex.c)
add_library(rtkpos  src/rtkpos.c)
add_library(ephemeris  src/ephemeris.c)
target_link_libraries(rtkpos lambda ppp pntpos ephemeris)
add_library(postpos  src/postpos.c)
target_link_libraries(postpos sbas rtkpos rtcm rtcm2 rtcm3 rtcm3e rinex)

add_library(lambda  src/lambda.c)
add_library(preceph  src/preceph.c)
add_library(ionex src/ionex.c)
add_library(pntpos  src/pntpos.c)
target_link_libraries(pntpos ionex)
add_library(options src/options.c)
add_library(ppp  src/ppp.c)
target_link_libraries(ppp pntpos ppp_ar)
add_library(rtcm src/rtcm.c)
add_library(rtcm2 src/rtcm2.c)
add_library(rtcm3 src/rtcm3.c)
add_library(rtcm3e src/rtcm3e.c)
add_library(qzslex src/qzslex.c)
add_library(ppp_ar src/ppp_ar.c)



add_definitions("-Wall -ansi -pedantic -I$(SRC) -DTRACE -lpthread
 -DENAGLO -DENAQZS -DENAGAL -DNFREQ=3")
include_directories ("${PROJECT_SOURCE_DIR}/src")
#rnx2rtkp

#add_executable(rtklibtest app/rnx2rtkp/rnx2rtkp.cpp)   
#target_link_libraries(rtklibtest ${catkin_LIBRARIES})  
#target_link_libraries(rtklibtest rtkcmn postpos solution ionex
#geoid preceph pntpos ephemeris options rtcm rtcm2 rtcm3 rtcm3e qzslex ppp_ar ionex)
#add_dependencies(rtklibtest robot_vision_generate_messages_cpp)  

add_executable(shadowmatching_node app/shadowmatching/shadowmatching_node.cpp)   
target_link_libraries(shadowmatching_node ${catkin_LIBRARIES})  
target_link_libraries(shadowmatching_node rtkcmn postpos solution ionex
geoid preceph pntpos ephemeris options rtcm rtcm2 rtcm3 rtcm3e qzslex ppp_ar ionex)
add_dependencies(shadowmatching_node robot_vision_generate_messages_cpp)  

add_executable(generateSkymask_node app/generate_skymask/generateSkymask_node.cpp)   
target_link_libraries(generateSkymask_node ${catkin_LIBRARIES})  
target_link_libraries(generateSkymask_node rtkcmn postpos solution ionex
geoid preceph pntpos ephemeris options rtcm rtcm2 rtcm3 rtcm3e qzslex ppp_ar ionex)
add_dependencies(generateSkymask_node robot_vision_generate_messages_cpp)  


