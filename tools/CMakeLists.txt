# Generate the traversal benchmark utility, and the ../common/traversal.h interface
add_subdirectory(bench_traversal)
add_subdirectory(bench_shading)
add_subdirectory(bench_interface)

find_package(CUDA QUIET)
if (CUDA_FOUND)
    add_subdirectory(bench_aila)
endif()

add_subdirectory(ray_gen)
find_package(PNG QUIET)
if (PNG_FOUND)
    add_subdirectory(fbuf2png)
endif()

add_subdirectory(bvh_extractor)

if (EMBREE_FOUND)
    add_subdirectory(bench_embree)
endif()

if (ImageMagick_FOUND AND PNG_FOUND)
    # Only test the primary rays, as the random rays are often too close
    # to surfaces and often give slightly different results for each algorithm
    add_test(NAME single_bvh4 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--single;--bvh-width;4" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=single-bvh4-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
    add_test(NAME packet_bvh4 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--packet;--ray-width;4;--bvh-width;4" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=packet-bvh4-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
    add_test(NAME hybrid_bvh4 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--ray-width;4;--bvh-width;4" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=hybrid-bvh4-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
    add_test(NAME single_bvh8 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--single;--bvh-width;8" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=single-bvh8-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
    add_test(NAME packet_bvh8 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--packet;--ray-width;8;--bvh-width;8" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=packet-bvh8-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
    add_test(NAME hybrid_bvh8 COMMAND ${CMAKE_COMMAND} -DBENCH_TRAVERSAL=$<TARGET_FILE:bench_traversal> -DFBUF2PNG=$<TARGET_FILE:fbuf2png> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DBENCH_TRAVERSAL_ARGS=--ray-width;8;--bvh-width;8" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DTRAVERSAL_OUTPUT=hybrid-bvh8-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_traversal.cmake)
endif()
