add_executable(ray_gen ray_gen.cpp ../common/traversal.h)
target_include_directories(ray_gen PUBLIC ../common)
target_link_libraries(ray_gen PUBLIC ${ANYDSL_RUNTIME_LIBRARY} ${ANYDSL_RUNTIME_LIBRARIES})
