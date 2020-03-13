cmake_minimum_required (VERSION 3.5)

project(UnscentedKalmanFilter})

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")

set(sources src/UKF_main.cpp src/tools.cpp src/UKF.cpp src/tools.h src/UKF.h)

add_executable(UnscentedKF ${sources})




