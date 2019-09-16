cmake_minimum_required (VERSION 3.5)

set(this UnscentedKFMain)

project(${this})

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

include_directories(src)
add_subdirectory(src)

enable_testing()
add_subdirectory(googletest)

set(sources src/main_test.cpp src/tools.cpp src/ukf.cpp src/ukf.h src/tools.h)

add_library(${this} STATIC ${sources})
target_link_libraries(${this} z ssl uv uWS)

add_subdirectory(test)


