set(TRAVERSAL_SRCS
    bench_traversal.impala
    ../../src/traversal/traversal.impala
    ../../src/traversal/intersection.impala
    ../../src/traversal/stack.impala
    ../../src/traversal/mapping_cpu.impala
    ../../src/traversal/mapping_gpu.impala
    ../../src/traversal/ray_layout.impala
    ../../src/core/common.impala
    ../../src/core/matrix.impala
    ../../src/core/vector.impala)

anydsl_runtime_wrap(TRAVERSAL_OBJS
    NAME "bench_traversal"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${TRAVERSAL_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/traversal)

add_executable(bench_traversal
    ${TRAVERSAL_OBJS}
    bench_traversal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/traversal)
target_include_directories(bench_traversal PUBLIC ../common)
target_link_libraries(bench_traversal ${ANYDSL_RUNTIME_LIBRARY} ${ANYDSL_RUNTIME_LIBRARIES})
add_custom_command(TARGET bench_traversal POST_BUILD COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
