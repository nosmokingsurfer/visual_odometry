cmake_minimum_required(VERSION 2.8)


project(visual_odometry)

FIND_PACKAGE(OpenCV REQUIRED)
find_package(OpenGV REQUIRED)
find_package(PCL 1.8.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

set(visual_HEADERS
    odometer.h
    camera.h
    helpers.h)
    
set(visual_SOURCE
    camera.cpp
    odometer.cpp
    main.cpp)


add_executable(visual_odometry ${visual_HEADERS} ${visual_SOURCE})
    
target_link_libraries(visual_odometry opengv ${OpenCV_LIBS} ${PCL_LIBRARIES})