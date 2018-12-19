set(SHADING_SRCS
    bench_shading.impala
    ../../src/render/mapping_cpu.impala
    ../../src/render/material.impala
    ../../src/render/geometry.impala
    ../../src/render/light.impala
    ../../src/render/image.impala
    ../../src/render/camera.impala
    ../../src/render/scene.impala
    ../../src/render/renderer.impala
    ../../src/render/driver.impala
    ../../src/traversal/intersection.impala
    ../../src/traversal/mapping_cpu.impala
    ../../src/traversal/mapping_gpu.impala
    ../../src/traversal/stack.impala
    ../../src/core/common.impala
    ../../src/core/cpu_common.impala
    ../../src/core/sort.impala
    ../../src/core/color.impala
    ../../src/core/random.impala
    ../../src/core/matrix.impala
    ../../src/core/vector.impala)

anydsl_runtime_wrap(SHADING_OBJS
    NAME "bench_shading"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${SHADING_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/shading)

add_executable(bench_shading
    ${SHADING_OBJS}
    bench_shading.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/shading.h)
target_include_directories(bench_shading PUBLIC ../common ../../src/driver)
target_link_libraries(bench_shading ${AnyDSL_runtime_LIBRARIES})
