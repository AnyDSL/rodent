# Generate the traversal benchmark utility, and the ../common/traversal.h interface
add_subdirectory(bench_traversal)
add_subdirectory(bench_shading)

find_package(CUDA QUIET)
if (CUDA_FOUND)
    add_subdirectory(bench_aila)
endif()

add_subdirectory(ray_gen)
find_package(PNG QUIET)
if (PNG_FOUND)
    add_subdirectory(fbuf2png)
endif()

if (EMBREE_FOUND)
    add_subdirectory(bvh_extractor)
    add_subdirectory(bench_embree)
endif()
