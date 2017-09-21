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
