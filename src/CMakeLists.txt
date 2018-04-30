set(CPUINFO_PATH "/proc/cpuinfo" CACHE STRING "Path to the CPU info file on the system")
try_run(CPUID_SUCCESS
        CPUID_COMPILE_SUCCESS
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/cpuid.cpp
        CMAKE_FLAGS -DCMAKE_CXX_STANDARD=11
        RUN_OUTPUT_VARIABLE CPUID_RESULT
        COMPILE_DEFINITIONS -DCPUINFO_PATH=\"${CPUINFO_PATH}\")
if (NOT CPUID_COMPILE_SUCCESS)
    message(FATAL_ERROR "Cannot determine CPU architecture!")
endif()
if (NOT CPUID_SUCCESS EQUAL 0)
    message(FATAL_ERROR "CPU does not have vector units!")
endif()
message(STATUS "CPU features found: ${CPUID_RESULT}")

set(RODENT_SRCS
    main.impala
    core/color.impala
    core/common.impala
    core/image.impala
    core/matrix.impala
    core/random.impala
    core/sort.impala
    core/vector.impala
    render/camera.impala
    render/geometry.impala
    render/light.impala
    render/material.impala
    render/renderer.impala
    render/scene.impala
    render/mapping_cpu.impala
    traversal/intersection.impala
    traversal/ray_layout.impala
    traversal/stack.impala
    traversal/mapping_cpu.impala
    traversal/mapping_gpu.impala)

anydsl_runtime_wrap(RODENT_OBJS
    NAME "rodent"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${RODENT_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/interface)

add_executable(rodent ${RODENT_OBJS} driver.cpp interface.h)
target_link_libraries(rodent ${AnyDSL_runtime_LIBRARIES})
