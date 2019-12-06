cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
project(3dPoseEstimation)
#set (TARGET_NAME 3dPoseEstimation)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(HEADER_FILES poseEstimation.hpp)

add_executable(GlobalAllignment main.cpp poseEstimation.cpp)


target_link_libraries(GlobalAllignment ${PCL_LIBRARIES})
#target_include_directories(${TARGET_NAME} PRIVATE include)
#target_link_libraries(ex2 ${PCL_LIBRARIES})

