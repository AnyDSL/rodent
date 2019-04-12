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
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${INTERFACE_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/interface)

add_executable(bench_interface
    ${INTERFACE_OBJS}
    bench_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/interface.h)
target_include_directories(bench_interface PUBLIC ../common ../../src/driver)
target_link_libraries(bench_interface ${AnyDSL_runtime_LIBRARIES})
