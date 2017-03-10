# Try to find Embree
find_path(EMBREE_ROOT_DIR include/embree2/rtcore.h DOC "Embree source code directory")
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
    add_executable(bvh_extractor
        bvh_extractor.cpp
        extract_bvh4.cpp
        extract_bvh2.cpp
        load_obj.cpp
        load_obj.h
        ../../src/traversal/frontend/traversal.h
        ../common/file_path.h
        ../common/float2.h
        ../common/float3.h
        ../common/bbox.h)
    target_include_directories(bvh_extractor PUBLIC ../common ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR})

    include(${EMBREE_CMAKE_DIR}/embree-config.cmake)
    if (EMBREE_TASKING_SYSTEM STREQUAL "TBB")
        target_compile_definitions(bvh_extractor PUBLIC -DTASKING_TBB)
        find_library(TBB_LIBRARY NAMES tbb.dll libtbb.so HINTS /usr/lib /usr/lib64)
        find_library(TBB_MALLOC_LIBRARY NAMES tbbmalloc.dll libtbbmalloc.so HINTS /usr/lib /usr/lib64)
        target_link_libraries(bvh_extractor ${TBB_LIBRARY})
        target_link_libraries(bvh_extractor ${TBB_MALLOC_LIBRARY})
    elseif (EMBREE_TASKING_SYSTEM STREQUAL "PPL")
        target_compile_definitions(bvh_extractor PUBLIC -DTASKING_PPL)
    else ()
        target_compile_definitions(bvh_extractor PUBLIC -DTASKING_INTERNAL)
    endif()

    find_library(EMBREE_SHARED_LIBRARY  NAMES embree.dll       libembree.so      HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree library")
    find_library(EMBREE_SSE42_LIBRARY   NAMES embree_sse42.lib libembree_sse42.a HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree SSE42 library")
    find_library(EMBREE_AVX_LIBRARY     NAMES embree_avx.lib   libembree_avx.a   HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX library")
    find_library(EMBREE_AVX2_LIBRARY    NAMES embree_avx2.lib  libembree_avx2.a  HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX2 library")
    find_library(EMBREE_SIMD_LIBRARY    NAMES simd.lib         libsimd.a         HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree SIMD library")
    find_library(EMBREE_TASKING_LIBRARY NAMES tasking.lib      libtasking.a      HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree tasking library")
    find_library(EMBREE_SYS_LIBRARY     NAMES sys.lib          libsys.a          HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree system library")
    find_library(PTHREAD_LIBRARY        NAMES pthread.dll      libpthread.so     HINTS /usr/lib /usr/lib64)

    if (PTHREAD_LIBRARY STREQUAL "PTHREAD_LIBRARY-NOTFOUND")
        # pthreads is not required on Windows
        set(PTHREADS_LIBRARY "")
    endif()

    target_link_libraries(bvh_extractor
        ${EMBREE_SSE42_LIBRARY}
        ${EMBREE_AVX_LIBRARY}
        ${EMBREE_AVX2_LIBRARY}
        ${EMBREE_TASKING_LIBRARY}
        ${EMBREE_SYS_LIBRARY}
        ${EMBREE_SIMD_LIBRARY}
        ${EMBREE_SHARED_LIBRARY}
        ${PTHREAD_LIBRARY})
endif()
