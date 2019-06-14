project(rodent)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

cmake_minimum_required(VERSION 3.1)

find_package(AnyDSL_runtime REQUIRED)
include_directories(${AnyDSL_runtime_INCLUDE_DIRS})

set(CLANG_FLAGS -O3 -march=native -ffast-math CACHE STRING "Clang compilation options")

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

set(COLOR_TTY_AVAILABLE TRUE)
if (WIN32)
    # By default, Windows console does not support ANSI escape codes
    set(COLOR_TTY_AVAILABLE FALSE)
endif ()
set(COLORIZE ${COLOR_TTY_AVAILABLE} CACHE BOOL "Set to TRUE to enable colorized output. Requires an ANSI compliant terminal.")

# Try to find Embree
find_path(EMBREE_ROOT_DIR include/embree3/rtcore.h DOC "Embree source code directory")
find_path(EMBREE_LIBRARY_DIR
    NAMES
    embree_sse42.dll
    embree_avx.dll
    embree_avx2.dll
    libembree_sse42.so
    libembree_avx.so
    libembree_avx2.so
    libembree_sse42.a
    libembree_avx.a
    libembree_avx2.a
    HINTS ${EMBREE_ROOT_DIR}/build
    DOC "Embree library directory")
find_path(EMBREE_CMAKE_DIR
    NAMES
    embree-config.cmake
    HINTS ${EMBREE_LIBRARY_DIR}
    DOC "Embree CMake directory")

if (NOT ${EMBREE_ROOT_DIR}    STREQUAL "EMBREE_ROOT_DIR-NOTFOUND"    AND
    NOT ${EMBREE_LIBRARY_DIR} STREQUAL "EMBREE_LIBRARY_DIR-NOTFOUND" AND
    NOT ${EMBREE_CMAKE_DIR}   STREQUAL "EMBREE_CMAKE_DIR-NOTFOUND")
    set(EMBREE_VERSION 3)
    find_library(EMBREE3_SHARED_LIBRARY NAMES embree3.dll libembree3.so HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree library")
    if (EMBREE3_SHARED_LIBRARY)
        set(EMBREE_SHARED_LIBRARY ${EMBREE3_SHARED_LIBRARY})
    else ()
        set(EMBREE_VERSION 2)
        find_library(EMBREE2_SHARED_LIBRARY NAMES embree.dll libembree.so HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree library")
        set(EMBREE_SHARED_LIBRARY ${EMBREE2_SHARED_LIBRARY})
    endif()
    add_definitions(-DEMBREE_VERSION=${EMBREE_VERSION})

    find_library(EMBREE_SSE42_LIBRARY   NAMES embree_sse42.lib libembree_sse42.a HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree SSE42 library")
    find_library(EMBREE_AVX_LIBRARY     NAMES embree_avx.lib   libembree_avx.a   HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX library")
    find_library(EMBREE_AVX2_LIBRARY    NAMES embree_avx2.lib  libembree_avx2.a  HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX2 library")
    find_library(EMBREE_SIMD_LIBRARY    NAMES simd.lib         libsimd.a         HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree SIMD library")
    find_library(EMBREE_TASKING_LIBRARY NAMES tasking.lib      libtasking.a      HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree tasking library")
    find_library(EMBREE_SYS_LIBRARY     NAMES sys.lib          libsys.a          HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree system library")

    find_library(PTHREAD_LIBRARY        NAMES pthread.dll      libpthread.so     HINTS /usr/lib /usr/lib64)
    find_library(TBB_LIBRARY            NAMES tbb.dll          libtbb.so         HINTS /usr/lib /usr/lib64)
    find_library(TBB_MALLOC_LIBRARY     NAMES tbbmalloc.dll    libtbbmalloc.so   HINTS /usr/lib /usr/lib64)

    set(EMBREE_DEPENDENCIES
        ${EMBREE_SHARED_LIBRARY}
        ${EMBREE_SSE42_LIBRARY}
        ${EMBREE_AVX_LIBRARY}
        ${EMBREE_AVX2_LIBRARY}
        ${EMBREE_TASKING_LIBRARY}
        ${EMBREE_SYS_LIBRARY}
        ${EMBREE_SIMD_LIBRARY})

    if (NOT PTHREAD_LIBRARY STREQUAL "PTHREAD_LIBRARY-NOTFOUND")
        # pthreads is not required on Windows
        set(EMBREE_DEPENDENCIES ${EMBREE_DEPENDENCIES} ${PTHREAD_LIBRARY})
    endif()

    function(get_embree_tasking_system defs libs)
        include(${EMBREE_CMAKE_DIR}/embree-config.cmake)
        set(${defs} "" PARENT_SCOPE)
        set(${libs} "" PARENT_SCOPE)
        if (EMBREE_TASKING_SYSTEM STREQUAL "TBB")
            set(${defs} -DTASKING_TBB PARENT_SCOPE)
            set(${libs} ${TBB_LIBRARY} ${TBB_MALLOC_LIBRARY} PARENT_SCOPE)
        elseif (EMBREE_TASKING_SYSTEM STREQUAL "PPL")
            set(${defs} -DTASKING_PPL PARENT_SCOPE)
        else ()
            set(${defs} -DTASKING_INTERNAL PARENT_SCOPE)
        endif()
    endfunction()

    get_embree_tasking_system(EMBREE_DEFINITIONS EMBREE_TASKING_DEPENDENCIES)
    set(EMBREE_DEPENDENCIES ${EMBREE_DEPENDENCIES} ${EMBREE_TASKING_DEPENDENCIES})

    message(STATUS "Embree found")
    set(EMBREE_FOUND ON)
endif()

# For tests
include(CTest)
find_package(ImageMagick COMPONENTS compare QUIET)

add_subdirectory(src)
add_subdirectory(tools)
