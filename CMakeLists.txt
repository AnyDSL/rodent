project(rodent)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

cmake_minimum_required(VERSION 3.1)

find_package(AnyDSL_runtime REQUIRED)
include_directories(${AnyDSL_runtime_INCLUDE_DIRS})

set(CLANG_FLAGS -O3 -mavx2 -mavx -mfma -ffast-math CACHE STRING "Clang compilation options")

set(CMAKE_CXX_STANDARD 11)

add_subdirectory(src)
add_subdirectory(tools)
