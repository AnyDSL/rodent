set(CLANG_FLAGS -mavx2 -mavx -mfma -ffast-math CACHE ON "CLANG compilation options")

set(TRAVERSAL_SRCS
    traversal/traversal.impala
    traversal/bbox.impala
    traversal/intersection.impala
    traversal/stack.impala
    traversal/mapping_cpu.impala
    traversal/mapping_gpu.impala
    traversal/frontend.impala
    core/common.impala
    core/matrix.impala
    core/vector.impala)

anydsl_runtime_wrap(TRAVERSAL_OBJS
                    NAME "traversal"
                    CLANG_FLAGS ${CLANG_FLAGS}
                    FILES ${TRAVERSAL_SRCS}
                    INTERFACE traversal/frontend/traversal)

add_library(rodent_traversal ${TRAVERSAL_OBJS})
set_target_properties(rodent_traversal PROPERTIES LINKER_LANGUAGE CXX)

add_executable(traversal_frontend traversal/frontend/frontend.cpp
                                  traversal/frontend/traversal.h)
target_link_libraries(traversal_frontend rodent_traversal ${ANYDSL_RUNTIME_LIBRARY} ${ANYDSL_RUNTIME_LIBRARIES})
