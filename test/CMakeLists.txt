cmake_minimum_required (VERSION 3.5)

set(this UKFMethodsTests)

set(sources TestUKF.cpp)

add_executable(${this} ${sources})

target_link_libraries(${this} PUBLIC gtest_main UnscentedKFMain)

add_test(
    NAME ${this}
    COMMAND ${this}
)

set(this UKFClassTests)

set(sources UKFClassTests.cpp)

add_executable(${this} ${sources})

target_link_libraries(${this} PUBLIC gtest_main UnscentedKFMain)

add_test(
    NAME ${this}
    COMMAND ${this}
)


