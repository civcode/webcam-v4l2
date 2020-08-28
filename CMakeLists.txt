cmake_minimum_required(VERSION 2.9)

project(webcam)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(grab grab.cpp webcam.cpp)

target_link_libraries(grab ${OpenCV_LIBS})

