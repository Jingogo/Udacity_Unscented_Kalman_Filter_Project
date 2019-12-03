cmake_minimum_required (VERSION 3.5)

project(KalmanFilter})

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")

set(sources src/UKF_main.cpp src/tools.cpp src/UKF.cpp src/tools.h src/UKF.h)

add_executable(ExtendedKF ${sources})
target_link_libraries(ExtendedKF z ssl uv uWS)

set(sources src/EKF_main.cpp src/tools.cpp src/EKF.cpp src/tools.h src/EKF.h)

add_executable(UnscentedKF ${sources})
target_link_libraries(UnscentedKF z ssl uv uWS)
