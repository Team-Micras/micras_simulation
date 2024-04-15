# Name: linter.cmake
# Micras Team
# Brief: This file checks if  CMake build type is correctly configured
# 04/2023

###############################################################################
## Build Type Check
###############################################################################

if(NOT (BUILD_TYPE STREQUAL "Release"        OR BUILD_TYPE STREQUAL "Debug" OR
        BUILD_TYPE STREQUAL "RelWithDebInfo" OR BUILD_TYPE STREQUAL "MinSizeRel"))
    set(BUILD_TYPE "RelWithDebInfo")
endif()
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
