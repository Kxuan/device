cmake_minimum_required(VERSION 3.14)
project(device)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
find_package(Boost REQUIRED COMPONENTS iostreams system)
include_directories(${Boost_INCLUDE_DIRS})

set(DEFAULT_PORT_NUMBER 6610 CACHE STRING "The default socket port")

configure_file(config.h.in config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(device main.cpp Port.cpp Port.hpp)

target_link_libraries(device ${Boost_LIBRARIES})