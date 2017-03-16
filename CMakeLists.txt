cmake_minimum_required(VERSION 2.6)
project(AnimX)

# set( CMAKE_VERBOSE_MAKEFILE on )

if (MAYA_64BIT_TIME_PRECISION)
    add_definitions(-DMAYA_64BIT_TIME_PRECISION)
endif()

if (APPLE AND ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -stdlib=libc++")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_subdirectory(src)
add_subdirectory(tests)