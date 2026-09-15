cmake_minimum_required(VERSION 3.14)
project(test_encoder_unwrap)

set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

add_executable(test_encoder_unwrap test_encoder_unwrap.cpp)
target_link_libraries(test_encoder_unwrap GTest::GTest GTest::Main)
