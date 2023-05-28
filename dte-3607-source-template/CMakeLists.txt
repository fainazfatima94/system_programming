# mimimum required version 3.15: policy CMP0092
# mimimum required version 3.12: target_compile_features C++ 20
# mimimum required version 3.8:  target_compile_features C++ 17
cmake_minimum_required(VERSION 3.15)

# policy CMP0092 MSVC COMPILE FLAGS -- no warings by default
cmake_policy(SET CMP0092 NEW)

# Project name
project(dte3607_2023_candidate_id VERSION 0.1 LANGUAGES CXX)

# Ease of setup protection on Wintendo OSes
if(MSVC AND NOT VCPKG_TOOLCHAIN)
  message(FATAL_ERROR
    "Enable VCPKG toolchain on MSVC platform\n"
    "  Star over; CMAKE_TOOLCHAIN_FILE definition missing!\n"
    "  i.e.: cmake -DCMAKE_TOOLCHAIN_FILE=<path_to>\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake <path_to>\\source\\"
    )
endif()

## Global compiler spesific compile options
SET(CLANG_COMPILE_OPTIONS
    # REQUIRED ON
    -Weverything -pedantic -Werror
    -Wno-c++98-compat -Wno-c++98-compat-pedantic
    # REQUIRED ON

    # Turn off warning for undocumented code fragments
    -Wno-documentation
  )

SET(GCC_COMPILE_OPTIONS
    # REQUIRED ON
    -pedantic -Wall -Werror
    # REQUIRED ON
  )

SET(MSVC_COMPILE_OPTIONS
    # REQUIRED ON
    /WX /W4 /permissive-
    # REQUIRED ON
  )

## Include subprojects
add_subdirectory(3rdparty/gmlib)
add_subdirectory(physengine)
add_subdirectory(testing)
add_subdirectory(benchmarking)


###########################################
# Qt creator dummy target
option(DUMMY_TARGET "Dummy target..." TRUE)
if(DUMMY_TARGET)
  add_custom_target(${PROJECT_NAME}_DUMMY)
endif(DUMMY_TARGET)


