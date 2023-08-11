macro(test filename)
    add_executable(${filename} ${filename}.cpp)
    target_link_libraries(${filename} reconstruct_pose)
    target_include_directories(${filename} PUBLIC ${CMAKE_SOURCE_DIR}/include)
    add_test(NAME ${filename} COMMAND ${filename})
endmacro()

test(triangulate_points)