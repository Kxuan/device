cmake_minimum_required(VERSION 3.14)
project(device)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Boost REQUIRED COMPONENTS iostreams system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)

set(DEFAULT_PORT_NUMBER 6610 CACHE STRING "The default socket port")
configure_file(config.h.in config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

pkg_check_modules(GSTMM REQUIRED gstreamermm-1.0)
pkg_check_modules(GTKMM REQUIRED gtkmm-2.4)
include_directories(${GTKMM_INCLUDE_DIRS} ${GSTMM_INCLUDE_DIRS})
link_directories(${GTKMM_LIBRARY_DIRS} ${GSTMM_LIBRARY_DIRS})

add_executable(device main.cpp Port.cpp Port.hpp VideoBus.cpp VideoBus.hpp)
target_link_libraries(device ${GTKMM_LIBRARIES} ${GSTMM_LIBRARIES})
target_link_libraries(device ${Boost_LIBRARIES})