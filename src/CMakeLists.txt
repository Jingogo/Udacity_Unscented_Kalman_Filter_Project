cmake_minimum_required (VERSION 3.5)

set(this UnscentedKF)

set(sources main_test.cpp tools.cpp ukf.cpp ukf.h tools.h)

add_executable(${this} ${sources})

target_link_libraries(${this} z ssl uv uWS)

