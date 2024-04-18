cmake_minimum_required(VERSION 2.6)
project(AnimX)

# set( CMAKE_VERBOSE_MAKEFILE on )
OPTION ( MAYA_64BIT_TIME_PRECISION "Use the library with a version of Maya including or after Maya after 2017 update 3" 1 )
OPTION ( ANIMX_BUILD_MAYA_TESTSUITE "Build the Maya test suite" 1 )
OPTION ( ANIMX_BUILD_STATIC "Build a static version of the AnimX library." 1 )
OPTION ( ANIMX_BUILD_SHARED "Build a shared library version of the AnimX library." 1 )

if (MAYA_64BIT_TIME_PRECISION)
    add_definitions(-DMAYA_64BIT_TIME_PRECISION)
endif()

if (APPLE AND ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -stdlib=libc++")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

add_subdirectory(src)

if (ANIMX_BUILD_MAYA_TESTSUITE)
    add_subdirectory(tests)
endif()