set(INTERFACE_SRCS
    bench_interface.impala
    ../../src/render/material.impala
    ../../src/render/light.impala
    ../../src/render/image.impala
    ../../src/core/common.impala
    ../../src/core/color.impala
    ../../src/core/random.impala
    ../../src/core/matrix.impala
    ../../src/core/vector.impala)

anydsl_runtime_wrap(INTERFACE_OBJS
    NAME "bench_interface"
    FRONTEND "artic"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${INTERFACE_SRCS}
    INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/interface)

add_executable(bench_interface ${INTERFACE_OBJS} bench_interface.cpp ${CMAKE_CURRENT_BINARY_DIR}/interface.h)
target_include_directories(bench_interface PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ../../src/driver)
target_link_libraries(bench_interface PUBLIC ${AnyDSL_runtime_LIBRARIES})
