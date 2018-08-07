set(TRAVERSAL_SRCS
    bench_traversal.impala
    ../../src/traversal/intersection.impala
    ../../src/traversal/stack.impala
    ../../src/traversal/mapping_cpu.impala
    ../../src/traversal/mapping_gpu.impala
    ../../src/core/common.impala
    ../../src/core/cpu_common.impala
    ../../src/core/sort.impala
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
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/traversal.h)
target_include_directories(bench_traversal PUBLIC ../common)
target_link_libraries(bench_traversal ${AnyDSL_runtime_LIBRARIES})
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc)
    add_custom_command(TARGET bench_traversal POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()
