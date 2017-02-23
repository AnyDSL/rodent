set(CLANG_FLAGS -mavx2 -mavx -mfma -ffast-math CACHE ON "CLANG compilation options")

set(TRAVERSAL_SRCS
    traversal/bvh.impala
    traversal/ray.impala
    traversal/stack.impala
    traversal/mapping_cpu.impala
    common.impala
    vector.impala)

anydsl_runtime_wrap(TRAVERSAL_OBJS
                    NAME "traversal"
                    CLANG_FLAGS ${CLANG_FLAGS}
                    FILES ${TRAVERSAL_SRCS})

add_library(rodent_traversal ${TRAVERSAL_OBJS})
set_target_properties(rodent_traversal PROPERTIES LINKER_LANGUAGE CXX)
