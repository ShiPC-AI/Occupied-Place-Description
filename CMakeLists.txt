cmake_minimum_required(VERSION 3.0.2)
project(lidar_onloc)

set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(PCL REQUIRED)

#OpenMP
set(CMAKE_CXX_FLAGS "-Wall -fopenmp")
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "########## Found openmp ##########") 
    set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} ${OPENMP_C_FLAGS})
    set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} ${OPENMP_CXX_FLAGS})
else()
    message(FATAL_ERROR "Openmp not found!")
endif()


include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_subdirectory(dataset_player)
add_subdirectory(offline_mapping)
add_subdirectory(online_localization)