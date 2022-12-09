cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(hdmap_curb)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_library(hdmap_curb crop_cloud.cpp crop_cloud.h)

add_executable (crop_cloud crop_cloud.cpp)
add_executable (flatten_intensity flatten_intensity.cpp)
add_executable (flatten_z flatten_z.cpp)

target_link_libraries (crop_cloud ${PCL_LIBRARIES})
target_link_libraries (flatten_intensity ${PCL_LIBRARIES})
target_link_libraries( flatten_intensity ${OpenCV_LIBS} )
target_link_libraries (flatten_z ${PCL_LIBRARIES})
target_link_libraries (flatten_z ${OpenCV_LIBS})