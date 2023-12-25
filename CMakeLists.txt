cmake_minimum_required(VERSION 3.10)
project(pureINS VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(ThirdParty/eigen3)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions( -D_USE_MATH_DEFINES=True)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
        src/main.cpp
        src/algebra.cpp
        src/algorithm.cpp
        src/fileio.cpp
        src/frame.cpp)

if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
    message("Debug mode:${CMAKE_C_FLAGS_DEBUG}")
    add_executable(${PROJECT_NAME} ${SOURCE})
    add_compile_options(-Wall -Werror -Wstrict-prototypes -Wmissing-prototypes)
elseif(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Release"))
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
    message("Release mode:${CMAKE_C_FLAGS_RELEASE}")
    add_executable(${PROJECT_NAME} ${SOURCE})
else()
    message("MODE:${CMAKE_BUILD_TYPE}")
    message("MODE:${CMAKE_C_FLAGS_RELEASE}")
    add_executable(${PROJECT_NAME} ${SOURCE})
endif()
set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin)

#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#set(CPACK_GENERATOR "ZIP")
#include(CPack)

