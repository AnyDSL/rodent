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

find_library(EMBREE_LIBRARY       NAMES embree.dll libembree.so HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree library")
find_library(EMBREE_SSE42_LIBRARY NAMES embree_sse42.lib libembree_sse42.a HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree SSE42 library")
find_library(EMBREE_AVX_LIBRARY   NAMES embree_avx.lib   libembree_avx.a   HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX library")
find_library(EMBREE_AVX2_LIBRARY  NAMES embree_avx2.lib  libembree_avx2.a  HINTS ${EMBREE_LIBRARY_DIR} DOC "Embree AVX2 library")

if (NOT ${EMBREE_ROOT_DIR} STREQUAL "EMBREE_ROOT_DIR-NOTFOUND" AND
    NOT ${EMBREE_LIBRARY} STREQUAL "EMBREE_LIBRARY-NOTFOUND")
    add_executable(bvh_extractor bvh_extractor.cpp file_path.h float2.h float3.h load_obj.cpp load_obj.h)
    target_include_directories(bvh_extractor PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR})
    target_link_libraries(bvh_extractor ${EMBREE_LIBRARY} ${EMBREE_SSE42_LIBRARY} ${EMBREE_AVX_LIBRARY} ${EMBREE_AVX2_LIBRARY})
endif()
