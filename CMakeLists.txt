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
pkg_check_modules(GLIBMM REQUIRED glibmm-2.4)
include_directories(${GLIBMM_INCLUDE_DIRS} ${GSTMM_INCLUDE_DIRS})
link_directories(${GLIBMM_LIBRARY_DIRS} ${GSTMM_LIBRARY_DIRS})

add_executable(rpi-cam-streaming main.cpp Port.cpp Port.hpp VideoBus.cpp VideoBus.hpp)
target_link_libraries(rpi-cam-streaming ${GLIBMM_LIBRARIES} ${GSTMM_LIBRARIES})
target_link_libraries(rpi-cam-streaming ${Boost_LIBRARIES})

install(TARGETS rpi-cam-streaming RUNTIME DESTINATION bin)