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

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../common)
anydsl_runtime_wrap(TRAVERSAL_OBJS
    NAME "bench_traversal"
    FRONTEND "artic"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${TRAVERSAL_SRCS}
    INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/../common/traversal)

add_library(traversal_interface INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/../common/traversal.h)
target_include_directories(traversal_interface INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/../common)

add_executable(bench_traversal ${TRAVERSAL_OBJS} bench_traversal.cpp)
target_link_libraries(bench_traversal PUBLIC ${AnyDSL_runtime_LIBRARIES} traversal_interface)
target_include_directories(bench_traversal PUBLIC ../common)
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc)
    add_custom_command(TARGET bench_traversal POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()
