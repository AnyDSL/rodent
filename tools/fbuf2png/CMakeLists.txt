add_executable(fbuf2png fbuf2png.cpp)
target_include_directories(fbuf2png PUBLIC ${PNG_INCLUDE_DIRS})
target_link_libraries(fbuf2png ${PNG_LIBRARIES})
