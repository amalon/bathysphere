cmake_minimum_required(VERSION 2.6)
include_directories(../..)

# This one should compile and fail with runtime error
add_executable(asserttest      asserttest.cpp      ../assert.cpp)

# This one should compile but fail to link
add_executable(asserttest_link asserttest_link.cpp ../assert.cpp)

# This one should fail to compile
add_executable(asserttest_fail asserttest_fail.cpp ../assert.cpp)
