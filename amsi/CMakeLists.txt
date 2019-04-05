cmake_minimum_required(VERSION 2.8.3)
project(amsi)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11)
# add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
# set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  ndt_omp
  rospy
)

#### needed by GTSAM (Weiosng Wen)
find_package(Boost REQUIRED COMPONENTS
  thread
  system
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}

)

###### GTSAM STUFF STARTS HERE

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes) # Load build type flags and default to Debug mode
#include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
#include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
#set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
#add_library(${CONVENIENCE_LIB_NAME} STATIC include/car/car_lib.h src/car_lib.cpp)
#target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

###### GTSAM STUFF ENDS HER


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
  INCLUDE_DIRS include
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


#pos2kml
#add_library(binex include/rtklib/binex.c)
#add_library(convkml include/rtklib/convkml.c)
#add_library(rinex include/rtklib/rinex.c)
#add_library(rtkpos include/rtklib/rtkpos.c)



# nodelets
add_library(amsi_nodelet apps/amsi_nodelet.cpp)
target_link_libraries(amsi_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#add_library(shadow_matching_nodelet apps/shadow_matching_nodelet.cpp)
#target_link_libraries(shadow_matching_nodelet
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

add_library(globalmap_server_nodelet apps/globalmap_server_nodelet.cpp)
target_link_libraries(globalmap_server_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


#add_executable(rnx2rtkp apps/rnx2rtkp.cpp)   
#target_link_libraries(rnx2rtkp ${catkin_LIBRARIES})  
#add_dependencies(rnx2rtkp robot_vision_generate_messages_cpp)  

add_executable(buildingModelLoader apps/buildingModelLoader.cpp )   
target_link_libraries(buildingModelLoader ${catkin_LIBRARIES})  
add_dependencies(buildingModelLoader robot_vision_generate_messages_cpp)  

add_executable(iSAM2Fusion apps/iSAM2Fusion.cpp )   
target_link_libraries(iSAM2Fusion gtsam ${catkin_LIBRARIES})  
add_dependencies(iSAM2Fusion robot_vision_generate_messages_cpp)  

add_executable(gps_imu_loose_fg apps/gps_imu_loose_fg.cpp )   
target_link_libraries(gps_imu_loose_fg gtsam ${catkin_LIBRARIES})  
add_dependencies(gps_imu_loose_fg robot_vision_generate_messages_cpp)  

add_executable(gps_imu_tight_fg apps/gps_imu_tight_fg.cpp )   
target_link_libraries(gps_imu_tight_fg gtsam ${catkin_LIBRARIES})  
add_dependencies(gps_imu_tight_fg robot_vision_generate_messages_cpp)  

add_executable(gps_imu_loose_ekf apps/gps_imu_loose_ekf.cpp )   
target_link_libraries(gps_imu_loose_ekf ${catkin_LIBRARIES})  
add_dependencies(gps_imu_loose_ekf robot_vision_generate_messages_cpp)  

add_executable(gps_imu_tight_ekf apps/gps_imu_tight_ekf.cpp )   
target_link_libraries(gps_imu_tight_ekf ${catkin_LIBRARIES})  
add_dependencies(gps_imu_tight_ekf robot_vision_generate_messages_cpp)  

add_executable(MMT2019_dataAnalysis apps/MMT2019_dataAnalysis.cpp )   
target_link_libraries(MMT2019_dataAnalysis  ${catkin_LIBRARIES})  
add_dependencies(MMT2019_dataAnalysis robot_vision_generate_messages_cpp)  

add_executable(jdSensorFusion apps/jdSensorFusion.cpp )   
target_link_libraries(jdSensorFusion gtsam ${catkin_LIBRARIES})  
add_dependencies(jdSensorFusion robot_vision_generate_messages_cpp)  

add_executable(laneMarkingAndBoundariesLoader apps/laneMarkingAndBoundariesLoader.cpp )   
target_link_libraries(laneMarkingAndBoundariesLoader ${catkin_LIBRARIES})  
add_dependencies(laneMarkingAndBoundariesLoader robot_vision_generate_messages_cpp)  
