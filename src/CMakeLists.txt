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
    render/mapping_cpu.impala
    traversal/bvh.impala
    traversal/intersection.impala
    traversal/stack.impala
    traversal/mapping_cpu.impala
    traversal/mapping_gpu.impala)

set(DRIVER_SRCS
    driver/driver.cpp
    driver/interface.cpp
    driver/interface.h
    driver/load_obj.cpp
    driver/load_obj.h
    driver/bvh.h
    driver/float2.h
    driver/float3.h
    driver/float4.h
    driver/file_path.h
    driver/common.h
    driver/color.h)

anydsl_runtime_wrap(RODENT_OBJS
    NAME "rodent"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${RODENT_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/driver/interface)

find_package(SDL2 REQUIRED)
find_package(PNG REQUIRED)
find_package(TBB REQUIRED)

add_executable(rodent ${DRIVER_SRCS} ${RODENT_OBJS})
target_include_directories(rodent PUBLIC ${PNG_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS})
target_link_libraries(rodent ${AnyDSL_runtime_LIBRARIES} ${PNG_LIBRARIES} ${SDL2_LIBRARY} ${TBB_LIBRARIES})
