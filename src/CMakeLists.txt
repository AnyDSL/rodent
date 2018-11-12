set(CPUINFO_PATH "/proc/cpuinfo" CACHE STRING "Path to the CPU info file on the system")
set(TARGET_DEVICE "" CACHE STRING "Target device for the converter tool. Must be one of: sse42, avx, avx2, asimd, nvvm")
set(SCENE_FILE "" CACHE FILEPATH "Absolute path to an OBJ scene")
set(MAX_PATH_LEN "64" CACHE STRING "Maximum path length")
set(SPP "4" CACHE STRING "Samples per pixel")
if (SCENE_FILE STREQUAL "")
    message(FATAL_ERROR "Please specify a valid OBJ scene in the SCENE_FILE variable")
endif()
if (NOT TARGET_DEVICE STREQUAL "")
    set(TARGET_OPTION "--target" "${TARGET_DEVICE}")
else()
    set(TARGET_OPTION "")
endif()

set(RODENT_SRCS
    core/color.impala
    core/common.impala
    core/cpu_common.impala
    core/matrix.impala
    core/random.impala
    core/sort.impala
    core/vector.impala
    render/image.impala
    render/camera.impala
    render/geometry.impala
    render/light.impala
    render/material.impala
    render/renderer.impala
    render/scene.impala
    render/driver.impala
    render/mapping_cpu.impala
    render/mapping_gpu.impala
    traversal/intersection.impala
    traversal/stack.impala
    traversal/mapping_cpu.impala
    traversal/mapping_gpu.impala)

set(DRIVER_SRCS
    driver/driver.cpp
    driver/interface.cpp
    driver/interface.h
    driver/obj.cpp
    driver/obj.h
    driver/image.cpp
    driver/image.h
    driver/bvh.h
    driver/float2.h
    driver/float3.h
    driver/float4.h
    driver/file_path.h
    driver/common.h
    driver/color.h)

set(CONVERTER_SRCS
    driver/converter.cpp
    driver/obj.cpp
    driver/obj.h
    driver/file_path.h
    driver/interface.h
    driver/bvh.h)

anydsl_runtime_wrap(RODENT_OBJS
    NAME "rodent"
    CLANG_FLAGS ${CLANG_FLAGS}
    IMPALA_FLAGS --log-level info
    FILES ${RODENT_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/main.impala)

anydsl_runtime_wrap(DISCARD_TMP_OBJS
    NAME "interface"
    FILES ${RODENT_SRCS} dummy_main.impala
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/driver/interface)

find_package(SDL2 REQUIRED)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TBB REQUIRED)
find_package(LZ4 REQUIRED)

add_executable(converter ${CONVERTER_SRCS})
target_link_libraries(converter ${LZ4_LIBRARY})
target_compile_definitions(converter PUBLIC -DCPUINFO_PATH="${CPUINFO_PATH}")
if (COLORIZE)
    target_compile_definitions(converter PUBLIC -DCOLORIZE)
endif()
if (EMBREE_FOUND)
    target_include_directories(converter PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR})
    target_link_libraries(converter ${EMBREE_DEPENDENCIES})
    target_compile_definitions(converter PUBLIC ${EMBREE_DEFINITIONS} -DENABLE_EMBREE_BVH)
endif()

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/main.impala
    COMMAND converter ${SCENE_FILE} ${TARGET_OPTION} --max-path-len ${MAX_PATH_LEN} --samples-per-pixel ${SPP}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${SCENE_FILE} converter)

set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/main.impala PROPERTIES GENERATED TRUE)

add_library(driver ${DRIVER_SRCS})
if (EMBREE_FOUND)
    target_include_directories(driver PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR})
    target_link_libraries(driver ${EMBREE_DEPENDENCIES})
    target_compile_definitions(driver PUBLIC -DENABLE_EMBREE_DEVICE)
endif()

add_executable(rodent ${RODENT_OBJS})
target_include_directories(rodent PUBLIC ${PNG_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${LZ4_INCLUDE_DIR})
target_link_libraries(rodent driver ${AnyDSL_runtime_LIBRARIES} ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${SDL2_LIBRARY} ${TBB_LIBRARIES} ${LZ4_LIBRARY})
