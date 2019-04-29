project(cats)
cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

find_package(SDL2 REQUIRED)
find_package(CUDA REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(OptiX REQUIRED)
find_package(Embree REQUIRED)
find_package(ISPC REQUIRED)
find_package(TBB REQUIRED)

add_subdirectory(src)
