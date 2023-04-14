set(EXTRACTOR_SRCS
    bvh_extractor.cpp
    extract_bvh2.cpp
    ../../src/driver/obj.cpp
    ../../src/driver/obj.h
    ../../src/driver/bvh.h)

if (EMBREE_FOUND)
    list(APPEND EXTRACTOR_SRCS extract_bvh4_8.cpp ../../src/driver/embree_bvh.h)
endif ()

add_executable(bvh_extractor ${EXTRACTOR_SRCS})
target_include_directories(bvh_extractor PUBLIC ../common ../../src)
target_link_libraries(bvh_extractor PUBLIC traversal_interface)

if (EMBREE_FOUND)
    target_include_directories(bvh_extractor PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR})
    target_compile_definitions(bvh_extractor PUBLIC ${EMBREE_DEFINITIONS})
    target_link_libraries(bvh_extractor PUBLIC ${EMBREE_DEPENDENCIES})
    target_compile_definitions(bvh_extractor PUBLIC -DENABLE_EMBREE_BVH)
endif ()
