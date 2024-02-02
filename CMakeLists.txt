#
# Copyright (c) 2023 SICK AG, Waldkirch
#
# SPDX-License-Identifier: Unlicense

cmake_minimum_required(VERSION 3.8)

if(POLICY CMP0092)
    cmake_policy(SET CMP0092 NEW) # Enables clean /W4 override for MSVC
endif()

project(SampleVisionary
        VERSION 2.0.0
        LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
option(VISIONARY_SHARED_ENABLE_CODE_COVERAGE "Enable code coverage using gcov" OFF)
option(VISIONARY_SHARED_ENABLE_AUTOIP "Enables the SOPAS Auto-IP device scan and assign of ip code (needs boost's ptree)" ON)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(WIN32)
  # We support only LONGHORN and above (WIN7...)
  add_compile_definitions("_WIN32_WINNT=_WIN32_WINNT_LONGHORN")
endif()

# configure warnings
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(VISIONARY_SHARED_CFLAGS
    -Werror
    -Wall
    -Wextra
    -Wsign-conversion
    -Wsign-compare
    -Wfloat-equal
    -Wtype-limits
    -Wshadow
    -Wconversion-null
    -pedantic
    -Wcast-align
    -Wcast-qual
    -Wformat
    -Wformat-nonliteral
    -Wformat-security
    -Winit-self
    -Wuninitialized
    -Wmissing-declarations
    -Wlogical-op
    -Wmissing-include-dirs
    -Wold-style-cast
    -Wpointer-arith
    -Wunreachable-code
    -Wnormalized=nfc
    -Wwrite-strings
    -Wvla
    -fstrict-aliasing
    -Wstrict-overflow=2
    -Woverloaded-virtual
    -Wsign-promo
    -Wno-unused-variable
    -ffunction-sections
    -fdata-sections
  )
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  set(VISIONARY_SHARED_CFLAGS
    /W4
    /WX
    /w44060
    /w44063
    /w44065
    /w45204
    /w45205
    /w44181
    /w44203
    /w44301
    /w44303
    /w44318
    /w44327
    /w44328
    /w44413
    /w44482
    /w44854
    /w45054
    /w45055
    /w45056
    /w45062
    /w45219
    /wd4189  # variable initialized but not used
  )
endif() # compilers

### BUILD ###
add_subdirectory(sick_visionary_cpp_shared)

## Visionary-S sample ##
add_executable(SampleVisionaryS SampleVisionaryS/SampleVisionaryS.cpp)
target_compile_options(SampleVisionaryS PRIVATE ${VISIONARY_SHARED_CFLAGS})
target_link_libraries(SampleVisionaryS sick_visionary_cpp_shared)

## Visionary-T Mini samples ##
add_executable(SampleVisionaryTMini SampleVisionaryTMini/SampleVisionaryTMini.cpp)
target_compile_options(SampleVisionaryTMini PRIVATE ${VISIONARY_SHARED_CFLAGS})
target_link_libraries(SampleVisionaryTMini sick_visionary_cpp_shared)

add_executable(SampleVisionaryTMiniFrameGrabber SampleVisionaryTMini/SampleVisionaryTMini.cpp)
target_compile_options(SampleVisionaryTMiniFrameGrabber PRIVATE ${VISIONARY_SHARED_CFLAGS})
target_link_libraries(SampleVisionaryTMiniFrameGrabber sick_visionary_cpp_shared)

## Visionary AutoIP ##
if(VISIONARY_SHARED_ENABLE_AUTOIP)
  message(STATUS "SOPAS AutoIP samples are built")
  add_executable(SampleAutoIP SampleAutoIP/SampleAutoIPScan.cpp)
  target_compile_options(SampleAutoIP PRIVATE ${VISIONARY_SHARED_CFLAGS})
  target_link_libraries(SampleAutoIP sick_visionary_cpp_shared)

  message(STATUS "SOPAS assign ip samples are built")
  add_executable(SampleAssignIP SampleAssignIP/SampleAssignIP.cpp)
  target_compile_options(SampleAssignIP PRIVATE ${VISIONARY_SHARED_CFLAGS})
  target_link_libraries(SampleAssignIP sick_visionary_cpp_shared)
endif()
