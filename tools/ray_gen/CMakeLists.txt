add_executable(ray_gen ray_gen.cpp)
target_include_directories(ray_gen PUBLIC ../common ../../src)
target_link_libraries(ray_gen PUBLIC traversal_interface ${AnyDSL_runtime_LIBRARIES})
