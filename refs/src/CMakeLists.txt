include_directories(${OptiX_INCLUDE} ${CUDA_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${Embree_INCLUDE_DIR} ${TBB_INCLUDE_DIRS})

cuda_wrap_srcs(optix_path_tracer PTX OPTIX_GENERATED_FILES optix_path_tracer.cu)
add_executable(optix_path_tracer optix_path_tracer.cpp obj.cpp obj.h image.cpp ${OPTIX_GENERATED_FILES})
target_link_libraries(optix_path_tracer PUBLIC ${SDL2_LIBRARY} ${optix_LIBRARY} ${optixu_LIBRARY} ${PNG_LIBRARY} ${JPEG_LIBRARY})
target_compile_definitions(optix_path_tracer PUBLIC -DGENERATED_PTX_FILE="${OPTIX_GENERATED_FILES}")

set(ENABLE_TIMING FALSE CACHE BOOL "Enables/disables timing information for Embree path tracer")
add_executable(embree_path_tracer embree_path_tracer.cpp obj.cpp image.cpp ${CMAKE_CURRENT_BINARY_DIR}/embree_path_tracer.ispc.generated.o)
if (${ENABLE_TIMING})
    set(ISPC_FLAGS -DENABLE_TIMING)
    target_compile_definitions(embree_path_tracer PUBLIC -DENABLE_TIMING -DFORCE_SERIAL_TIMING)
endif()
add_custom_command(
    OUTPUT embree_path_tracer.ispc.generated.o
    COMMAND ${ISPC_DIR}/ispc ${CMAKE_CURRENT_SOURCE_DIR}/embree_path_tracer.ispc -o embree_path_tracer.ispc.generated.o -O3 --target=avx2-i32x8 -I${Embree_INCLUDE_DIR} ${ISPC_FLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${ISPC_DIR}/ispc ${CMAKE_CURRENT_SOURCE_DIR}/embree_path_tracer.ispc ${CMAKE_CURRENT_SOURCE_DIR}/common.isph ${CMAKE_CURRENT_SOURCE_DIR}/math.isph)
target_link_libraries(embree_path_tracer PUBLIC ${SDL2_LIBRARY} ${Embree_LIBRARY} ${PNG_LIBRARY} ${JPEG_LIBRARY} ${TBB_LIBRARIES})

add_executable(bench_shading bench_shading.cpp ${CMAKE_CURRENT_BINARY_DIR}/bench_shading.ispc.generated.o)
add_custom_command(
    OUTPUT bench_shading.ispc.generated.o
    COMMAND ${ISPC_DIR}/ispc ${CMAKE_CURRENT_SOURCE_DIR}/bench_shading.ispc -o bench_shading.ispc.generated.o -O3 --target=avx2-i32x8 -I${Embree_INCLUDE_DIR} ${ISPC_FLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${ISPC_DIR}/ispc ${CMAKE_CURRENT_SOURCE_DIR}/bench_shading.ispc ${CMAKE_CURRENT_SOURCE_DIR}/common.isph ${CMAKE_CURRENT_SOURCE_DIR}/math.isph)
