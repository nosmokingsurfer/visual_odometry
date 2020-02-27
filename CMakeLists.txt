cmake_minimum_required(VERSION 2.8)

project(visual_odometry)

FIND_PACKAGE(OpenCV 4.2.0 REQUIRED)
find_package(OpenGV REQUIRED)

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
    
target_link_libraries(visual_odometry opengv ${OpenCV_LIBS})